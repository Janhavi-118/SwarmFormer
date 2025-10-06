# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swarm_msgs:msg/Gossip.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Gossip(type):
    """Metaclass of message 'Gossip'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('swarm_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swarm_msgs.msg.Gossip')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gossip
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gossip
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gossip
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gossip
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gossip

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from swarm_msgs.msg import GMMParams
            if GMMParams.__class__._TYPE_SUPPORT is None:
                GMMParams.__class__.__import_type_support__()

            from swarm_msgs.msg import RobotState
            if RobotState.__class__._TYPE_SUPPORT is None:
                RobotState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Gossip(metaclass=Metaclass_Gossip):
    """Message class 'Gossip'."""

    __slots__ = [
        '_sender',
        '_timestamp',
        '_robot_state',
        '_known_neighbors',
        '_failed_robots',
        '_gmm_params',
        '_message_type',
    ]

    _fields_and_field_types = {
        'sender': 'string',
        'timestamp': 'builtin_interfaces/Time',
        'robot_state': 'swarm_msgs/RobotState',
        'known_neighbors': 'sequence<swarm_msgs/RobotState>',
        'failed_robots': 'sequence<string>',
        'gmm_params': 'swarm_msgs/GMMParams',
        'message_type': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['swarm_msgs', 'msg'], 'RobotState'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['swarm_msgs', 'msg'], 'RobotState')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['swarm_msgs', 'msg'], 'GMMParams'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sender = kwargs.get('sender', str())
        from builtin_interfaces.msg import Time
        self.timestamp = kwargs.get('timestamp', Time())
        from swarm_msgs.msg import RobotState
        self.robot_state = kwargs.get('robot_state', RobotState())
        self.known_neighbors = kwargs.get('known_neighbors', [])
        self.failed_robots = kwargs.get('failed_robots', [])
        from swarm_msgs.msg import GMMParams
        self.gmm_params = kwargs.get('gmm_params', GMMParams())
        self.message_type = kwargs.get('message_type', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.sender != other.sender:
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.robot_state != other.robot_state:
            return False
        if self.known_neighbors != other.known_neighbors:
            return False
        if self.failed_robots != other.failed_robots:
            return False
        if self.gmm_params != other.gmm_params:
            return False
        if self.message_type != other.message_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sender(self):
        """Message field 'sender'."""
        return self._sender

    @sender.setter
    def sender(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sender' field must be of type 'str'"
        self._sender = value

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp' field must be a sub message of type 'Time'"
        self._timestamp = value

    @builtins.property
    def robot_state(self):
        """Message field 'robot_state'."""
        return self._robot_state

    @robot_state.setter
    def robot_state(self, value):
        if __debug__:
            from swarm_msgs.msg import RobotState
            assert \
                isinstance(value, RobotState), \
                "The 'robot_state' field must be a sub message of type 'RobotState'"
        self._robot_state = value

    @builtins.property
    def known_neighbors(self):
        """Message field 'known_neighbors'."""
        return self._known_neighbors

    @known_neighbors.setter
    def known_neighbors(self, value):
        if __debug__:
            from swarm_msgs.msg import RobotState
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, RobotState) for v in value) and
                 True), \
                "The 'known_neighbors' field must be a set or sequence and each value of type 'RobotState'"
        self._known_neighbors = value

    @builtins.property
    def failed_robots(self):
        """Message field 'failed_robots'."""
        return self._failed_robots

    @failed_robots.setter
    def failed_robots(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'failed_robots' field must be a set or sequence and each value of type 'str'"
        self._failed_robots = value

    @builtins.property
    def gmm_params(self):
        """Message field 'gmm_params'."""
        return self._gmm_params

    @gmm_params.setter
    def gmm_params(self, value):
        if __debug__:
            from swarm_msgs.msg import GMMParams
            assert \
                isinstance(value, GMMParams), \
                "The 'gmm_params' field must be a sub message of type 'GMMParams'"
        self._gmm_params = value

    @builtins.property
    def message_type(self):
        """Message field 'message_type'."""
        return self._message_type

    @message_type.setter
    def message_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message_type' field must be of type 'str'"
        self._message_type = value
