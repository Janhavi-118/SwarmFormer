
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import numpy as np
import pickle
from pathlib import Path
import random
from tqdm import tqdm
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
import logging

# Logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

#############################
# CVaR Feature Extraction   #
#############################
def compute_cvar_lidar(scan_msg, alpha=0.05):
    if scan_msg is None:
        return 0.0
    if isinstance(scan_msg, list):
        if len(scan_msg) > 0:
            scan_msg = scan_msg[0]
        else:
            return 0.0
    if not hasattr(scan_msg, 'ranges'):
        return 0.0
    ranges = np.array(scan_msg.ranges)
    ranges = np.where(np.isinf(ranges), scan_msg.range_max, ranges)
    valid = ranges[np.isfinite(ranges)]
    if len(valid) == 0:
        return 1.0
    n_worst = max(1, int(alpha * len(valid)))
    cvar = np.sort(valid)[:n_worst].mean()
    risk = 1.0 - (cvar / scan_msg.range_max)
    return float(np.clip(risk, 0.0, 1.0))

#############################
# Model Architecture        #
#############################
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, max_len=5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        pos = torch.arange(0, max_len).unsqueeze(1).float()
        div = torch.exp(torch.arange(0, d_model, 2).float() * -(torch.log(torch.tensor(10000.0)) / d_model))
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        self.register_buffer('pe', pe.unsqueeze(0))
    def forward(self, x):
        return x + self.pe[:, :x.size(1)]

class ModalityEncoder(nn.Module):
    def __init__(self, inp_dim, d_model, nhead, layers, dropout=0.1):
        super().__init__()
        self.proj = nn.Linear(inp_dim, d_model)
        self.pos = PositionalEncoding(d_model)
        layer = nn.TransformerEncoderLayer(d_model, nhead, d_model*4, dropout, batch_first=True)
        self.enc = nn.TransformerEncoder(layer, layers)
        self.norm = nn.LayerNorm(d_model)
        self.drop = nn.Dropout(dropout)
    def forward(self, x, mask=None):
        x = self.proj(x) * (self.proj.out_features ** 0.5)
        x = self.pos(x)
        x = self.drop(x)
        x = self.enc(x, src_key_padding_mask=mask)
        return self.norm(x)

class CrossModalAttention(nn.Module):
    def __init__(self, d_model, nhead, dropout=0.1):
        super().__init__()
        self.attn = nn.MultiheadAttention(d_model, nhead, dropout, batch_first=True)
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.ff = nn.Sequential(
            nn.Linear(d_model, d_model*4),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(d_model*4, d_model)
        )
        self.drop = nn.Dropout(dropout)
    def forward(self, q, k, v, kmask=None):
        out, _ = self.attn(q, k, v, key_padding_mask=kmask)
        q = self.norm1(q + self.drop(out))
        ff = self.ff(q)
        return self.norm2(q + self.drop(ff))

class GMMHead(nn.Module):
    def __init__(self, d_model, act_dim, comps=5):
        super().__init__()
        self.K = comps
        self.ad = act_dim
        self.logits = nn.Linear(d_model, comps)
        self.means = nn.Linear(d_model, comps * act_dim)
        self.log_stds = nn.Linear(d_model, comps * act_dim)
    def forward(self, x):
        B, T, _ = x.shape
        logits = self.logits(x)
        means = self.means(x).view(B, T, self.K, self.ad)
        logstd = self.log_stds(x).view(B, T, self.K, self.ad)
        stds = torch.exp(logstd.clamp(-5,2))
        return logits, means, stds

class SwarmTransformer(nn.Module):
    def __init__(self):
        super().__init__()
        self.odom_enc = ModalityEncoder(13, 256, 8, 4)
        self.scan_enc = ModalityEncoder(361, 256, 8, 4)
        self.gossip_enc = ModalityEncoder(70, 256, 8, 4)
        self.cross1 = CrossModalAttention(256, 8)
        self.cross2 = CrossModalAttention(256, 8)
        self.gmm = GMMHead(256, 2, comps=5)
    def forward(self, odom, scan, gossip, smask, gmask):
        o = self.odom_enc(odom)
        s = self.scan_enc(scan, smask)
        g = self.gossip_enc(gossip, gmask)
        os = self.cross1(o, s, s, smask)
        full = self.cross2(os, g, g, gmask)
        return self.gmm(full)
    def compute_loss(self, logits, means, stds, tgt):
        B, T, K, D = means.shape
        lf = logits.view(-1, K)
        mf = means.view(-1, K, D)
        sf = stds.view(-1, K, D)
        tf = tgt.view(-1, D)
        mix = torch.distributions.Categorical(logits=lf)
        comp = torch.distributions.Independent(torch.distributions.Normal(mf, sf), 1)
        gmm = torch.distributions.MixtureSameFamily(mix, comp)
        return -gmm.log_prob(tf).mean()

#############################
# Dataset Class             #
#############################
class SwarmRobotDataset(Dataset):
    def __init__(self, paths, max_nb=10, scan_dim=360, seq_len=None, normalize=True):
        self.paths = paths
        self.max_nb = max_nb
        self.scan_dim = scan_dim
        self.seq_len = seq_len
        self.normalize = normalize
        self.recs = []
        self.load_all()
        if self.normalize:
            self.compute_stats()
    def load_all(self):
        for p in self.paths:
            try:
                ep = pickle.load(open(p,'rb'))
                for rn, rd in ep.items():
                    if len(rd['odom_ts'])>10:
                        self.recs.append((p, rn, rd))
            except:
                continue
    def compute_stats(self):
        o, s, g, c = [], [], [], []
        for i in random.sample(self.recs, min(len(self.recs),1000)):
            odom, scan, gossip, sm, gm, cmd = self.extract(i)
            o.append(odom); s.append(scan); g.append(gossip); c.append(cmd)
        o=torch.cat(o); s=torch.cat(s); g=torch.cat(g); c=torch.cat(c)
        self.om, self.os = o.mean(0), o.std(0)+1e-8
        self.sm, self.ss = s.mean(0), s.std(0)+1e-8
        self.gm, self.gs = g.mean(0), g.std(0)+1e-8
        self.cm, self.cs = c.mean(0), c.std(0)+1e-8
    def extract(self, rec):
        _,_,rd=rec
        L=len(rd['odom_ts'])
        odom=[];scan=[];sm=[];goss=[];gm=[];cmd=[]
        for i,msg in enumerate(rd['odom_data']):
            if isinstance(msg,list): msg=msg[0]
            p=msg.pose.pose; t=msg.twist.twist
            odom.append([p.position.x,p.position.y,p.position.z,
                         p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w,
                         t.linear.x,t.linear.y,t.linear.z,
                         t.angular.x,t.angular.y,t.angular.z])
        for i,msg in enumerate(rd['scan_filled']):
            masked=rd['scan_mask'][i]==1
            if masked or msg is None:
                scan.append([0.0]*(self.scan_dim+1)); sm.append(True)
            else:
                if isinstance(msg,list): msg=msg[0]
                cvar=compute_cvar_lidar(msg)
                r=np.array(msg.ranges); r=np.where(np.isinf(r), msg.range_max, r)
                r=r/msg.range_max
                if len(r)!=self.scan_dim:
                    r=np.interp(np.linspace(0,len(r)-1,self.scan_dim),np.arange(len(r)),r)
                scan.append([cvar,*r]); sm.append(False)
        for i,gd in enumerate(rd['gossip_filled']):
            masked=rd['gossip_mask'][i]==0
            arr=np.zeros(self.max_nb*7); gm.append(masked)
            if not masked and gd:
                for j,(k,v) in enumerate(gd.items()):
                    if j>=self.max_nb: break
                    arr[j*7:j*7+3]=v['position'][:3]
                    arr[j*7+3:j*7+7]=v['orientation'][:4]
            goss.append(arr)
        for msg in rd['cmd_vel_filled']:
            if isinstance(msg,list): msg=msg[0]
            cmd.append([msg.linear.x, msg.angular.z])
        odom,scan,goss,cmd = map(torch.tensor, (odom,scan,goss,cmd))
        sm,gm=map(lambda x:torch.tensor(x,dtype=torch.bool),(sm,gm))
        return odom,scan,goss,sm,gm,cmd
    def __len__(self): return len(self.recs)
    def __getitem__(self,idx):
        rec=self.recs[idx]
        odom,scan,goss,sm,gm,cmd=self.extract(rec)
        if self.seq_len and len(odom)>self.seq_len:
            i=random.randint(0,len(odom)-self.seq_len)
            odom,scan,goss,sm,gm,cmd=[x[i:i+self.seq_len] for x in (odom,scan,goss,sm,gm,cmd)]
        if self.normalize:
            odom=(odom-self.om)/self.os
            scan=(scan-self.sm)/self.ss
            goss=(goss-self.gm)/self.gs
            cmd=(cmd-self.cm)/self.cs
        return {'odom':odom,'scan':scan,'gossip':goss,'scan_mask':sm,'gossip_mask':gm,'cmd_vel':cmd}

def collate_fn(batch):
    B=len(batch)
    L=max(len(x['odom']) for x in batch)
    odom=torch.zeros(B,L,13); scan=torch.zeros(B,L,361)
    gossip=torch.zeros(B,L,70); sm=torch.ones(B,L,bool); gm=torch.ones(B,L,bool)
    cmd=torch.zeros(B,L,2)
    for i,x in enumerate(batch):
        l=len(x['odom'])
        odom[i,:l]=x['odom']; scan[i,:l]=x['scan']; gossip[i,:l]=x['gossip']
        sm[i,:l]=x['scan_mask']; gm[i,:l]=x['gossip_mask']; cmd[i,:l]=x['cmd_vel']
    return {'odom':odom,'scan':scan,'gossip':gossip,'scan_mask':sm,'gossip_mask':gm,'cmd_vel':cmd}

#############################
# Trainer                   #
#############################
class SwarmTrainer:
    def __init__(self, model, train_loader, val_loader, test_loader, device):
        self.m, self.tl, self.vl, self.test = model.to(device), train_loader, val_loader, test_loader
        self.dev=device
        self.opt=torch.optim.AdamW(self.m.parameters(),lr=1e-4,weight_decay=1e-5)
        self.sch=torch.optim.lr_scheduler.CosineAnnealingLR(self.opt,T_max=100)
        self.best=1e9; self.train_losses=[]; self.val_losses=[]
    def train_epoch(self):
        self.m.train(); tot=cnt=0
        for b in tqdm(self.tl,desc="Train"):
            self.opt.zero_grad()
            od,sc,go,sm,gm,cmd=[b[k].to(self.dev) for k in ('odom','scan','gossip','scan_mask','gossip_mask','cmd_vel')]
            lgs,ms,ss=self.m(od,sc,go,sm,gm)
            loss=self.m.compute_loss(lgs,ms,ss,cmd)
            loss.backward(); torch.nn.utils.clip_grad_norm_(self.m.parameters(),1.0)
            self.opt.step()
            tot+=loss.item(); cnt+=1
        return tot/cnt
    def eval_epoch(self):
        self.m.eval(); tot=cnt=0
        with torch.no_grad():
            for b in tqdm(self.vl,desc="Val"):
                od,sc,go,sm,gm,cmd=[b[k].to(self.dev) for k in ('odom','scan','gossip','scan_mask','gossip_mask','cmd_vel')]
                lgs,ms,ss=self.m(od,sc,go,sm,gm)
                loss=self.m.compute_loss(lgs,ms,ss,cmd)
                tot+=loss.item(); cnt+=1
        return tot/cnt
    def test_epoch(self):
        self.m.eval(); tot=cnt=0
        with torch.no_grad():
            for b in tqdm(self.test,desc="Test"):
                od,sc,go,sm,gm,cmd=[b[k].to(self.dev) for k in ('odom','scan','gossip','scan_mask','gossip_mask','cmd_vel')]
                lgs,ms,ss=self.m(od,sc,go,sm,gm)
                loss=self.m.compute_loss(lgs,ms,ss,cmd)
                tot+=loss.item(); cnt+=1
        return tot/cnt
    def train(self, epochs=100):
        for e in range(epochs):
            tr=self.train_epoch(); self.train_losses.append(tr)
            va=self.eval_epoch(); self.val_losses.append(va)
            if va<self.best:
                self.best=va
                torch.save(self.m.state_dict(),"swarm_transformer.pth")
            self.sch.step()
            logger.info(f"Epoch {e+1}/{epochs} Train {tr:.4f} Val {va:.4f}")
        tc=plt.figure(); plt.plot(self.train_losses,label='Train'); plt.plot(self.val_losses,label='Val')
        plt.legend(); plt.savefig("train_curve.png")
        te=self.test_epoch()
        logger.info(f"Test Loss: {te:.4f}")

#############################
# Main                      #
#############################
def main():
    DEVICE="cuda" if torch.cuda.is_available() else "cpu"
    DATA_DIR="/home/janhavi/swarm_ws/processed_datasets"
    files=list(Path(DATA_DIR).glob("*_dataset.pkl"))
    tr, temp = train_test_split(files,test_size=0.3,random_state=42)
    val, te = train_test_split(temp,test_size=0.5,random_state=42)
    logger.info(f"Train {len(tr)}, Val {len(val)}, Test {len(te)} episodes")
    train_ds=SwarmRobotDataset(tr,seq_len=200,normalize=True)
    val_ds  =SwarmRobotDataset(val,seq_len=200,normalize=True)
    test_ds =SwarmRobotDataset(te, seq_len=200,normalize=True)
    # share stats
    for ds in (val_ds,test_ds):
        ds.om,ds.os,ds.sm,ds.ss,ds.gm,ds.gs,ds.cm,ds.cs = (
            train_ds.om, train_ds.os, train_ds.sm, train_ds.ss,
            train_ds.gm, train_ds.gs, train_ds.cm, train_ds.cs
        )
    TL=DataLoader(train_ds,8,shuffle=True,collate_fn=collate_fn,num_workers=1)
    VL=DataLoader(val_ds,8,shuffle=False,collate_fn=collate_fn,num_workers=1)
    TE=DataLoader(test_ds,8,shuffle=False,collate_fn=collate_fn,num_workers=1)
    model=SwarmTransformer()
    logger.info(f"Params: {sum(p.numel() for p in model.parameters()):,}")
    trainer=SwarmTrainer(model,TL,VL,TE,DEVICE)
    trainer.train(epochs=100)

if __name__=="__main__":
    main()

'''
import torch
import torch.nn as nn
import torch.nn.functional as F

#############################
# CVaR Feature Extraction   #
#############################
def compute_cvar_lidar(scan_ranges, range_max, alpha=0.05):
    """Compute CVaR risk score from raw scan ranges."""
    ranges = torch.tensor(scan_ranges)
    ranges = torch.where(torch.isinf(ranges), range_max, ranges)
    valid = ranges[torch.isfinite(ranges)]
    if valid.numel() == 0:
        return torch.tensor(1.0)
    n_worst = max(1, int(alpha * valid.numel()))
    cvar = valid.sort().values[:n_worst].mean()
    risk = 1.0 - (cvar / range_max)
    return torch.clamp(risk, 0.0, 1.0)

#############################
# Model Architecture        #
#############################
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, max_len=5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        pos = torch.arange(max_len).unsqueeze(1).float()
        div = torch.exp(torch.arange(0, d_model, 2).float() * -(torch.log(torch.tensor(10000.0)) / d_model))
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        self.register_buffer('pe', pe.unsqueeze(0))
    def forward(self, x):
        return x + self.pe[:, :x.size(1)]

class ModalityEncoder(nn.Module):
    def __init__(self, inp_dim, d_model, nhead, layers, dropout=0.1):
        super().__init__()
        self.proj = nn.Linear(inp_dim, d_model)
        self.pos = PositionalEncoding(d_model)
        layer = nn.TransformerEncoderLayer(d_model, nhead, d_model*4, dropout, batch_first=True)
        self.enc = nn.TransformerEncoder(layer, layers)
        self.norm = nn.LayerNorm(d_model)
        self.drop = nn.Dropout(dropout)
    def forward(self, x, mask=None):
        x = self.proj(x) * (self.proj.out_features ** 0.5)
        x = self.pos(x)
        x = self.drop(x)
        x = self.enc(x, src_key_padding_mask=mask)
        return self.norm(x)

class CrossModalAttention(nn.Module):
    def __init__(self, d_model, nhead, dropout=0.1):
        super().__init__()
        self.attn = nn.MultiheadAttention(d_model, nhead, dropout, batch_first=True)
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.ff = nn.Sequential(
            nn.Linear(d_model, d_model*4),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(d_model*4, d_model)
        )
        self.drop = nn.Dropout(dropout)
    def forward(self, q, k, v, kmask=None):
        out, _ = self.attn(q, k, v, key_padding_mask=kmask)
        q = self.norm1(q + self.drop(out))
        ff = self.ff(q)
        return self.norm2(q + self.drop(ff))

class GMMHead(nn.Module):
    def __init__(self, d_model, act_dim, comps=5):
        super().__init__()
        self.K = comps
        self.ad = act_dim
        self.logits = nn.Linear(d_model, comps)
        self.means = nn.Linear(d_model, comps * act_dim)
        self.log_stds = nn.Linear(d_model, comps * act_dim)
    def forward(self, x):
        B, T, _ = x.shape
        logits = self.logits(x)                # [B, T, K]
        means = self.means(x).view(B, T, self.K, self.ad)
        logstd = self.log_stds(x).view(B, T, self.K, self.ad)
        stds = torch.exp(logstd.clamp(-5,2))
        return logits, means, stds

class SwarmTransformer(nn.Module):
    def __init__(self):
        super().__init__()
        self.odom_enc   = ModalityEncoder(13, 256, 8, 4)
        self.scan_enc   = ModalityEncoder(361, 256, 8, 4)
        self.gossip_enc = ModalityEncoder(70, 256, 8, 4)
        self.cross1     = CrossModalAttention(256, 8)
        self.cross2     = CrossModalAttention(256, 8)
        self.gmm        = GMMHead(256, 2, comps=5)
    def forward(self, odom, scan, gossip, smask, gmask):
        o = self.odom_enc(odom)
        s = self.scan_enc(scan, smask)
        g = self.gossip_enc(gossip, gmask)
        os = self.cross1(o, s, s, smask)
        full = self.cross2(os, g, g, gmask)
        return self.gmm(full)

#############################
# Dummy Data & Test Run     #
#############################
if __name__ == "__main__":
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = SwarmTransformer().to(device)

    # Dummy batch parameters
    B, T = 2, 100
    odom = torch.randn(B, T, 13, device=device)
    scan = torch.rand(B, T, 360, device=device) * 10.0  # ranges [0,10]
    gossip = torch.randn(B, T, 70, device=device)
    smask = torch.zeros(B, T, dtype=torch.bool, device=device)
    gmask = torch.zeros(B, T, dtype=torch.bool, device=device)

    # Insert CVaR as first element of scan vector
    # Here we simulate range_max=10.0
    scan_with_cvar = torch.zeros(B, T, 361, device=device)
    for b in range(B):
        for t in range(T):
            cvar = compute_cvar_lidar(scan[b,t].cpu().numpy(), 10.0).to(device)
            scan_with_cvar[b,t,0] = cvar
            scan_with_cvar[b,t,1:] = scan[b,t]

    # Dummy target cmd_vel
    target = torch.randn(B, T, 2, device=device)

    # Forward pass
    logits, means, stds = model(odom, scan_with_cvar, gossip, smask, gmask)
    print("Shapes:", logits.shape, means.shape, stds.shape)

    # Compute vectorized GMM loss
    K, D = means.shape[2], means.shape[3]
    lf = logits.view(-1, K)
    mf = means.view(-1, K, D)
    sf = stds.view(-1, K, D)
    tf = target.view(-1, D)
    mix = torch.distributions.Categorical(logits=lf)
    comp = torch.distributions.Independent(torch.distributions.Normal(mf, sf), 1)
    gmm = torch.distributions.MixtureSameFamily(mix, comp)
    loss = -gmm.log_prob(tf).mean()
    print("Dummy Loss:", loss.item())
'''