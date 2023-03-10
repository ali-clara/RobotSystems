# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from warehouse/Grasp.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import warehouse.msg

class Grasp(genpy.Message):
  _md5sum = "70d37fa314d53b4952d2f54caf3874d2"
  _type = "warehouse/Grasp"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# 夹取时的姿态和位置
warehouse/Pose grasp_pos

# 接近时的距离和方向
geometry_msgs/Vector3 grasp_approach

# 撤离时的距离和方向
geometry_msgs/Vector3 grasp_retreat

# 抬高高度
float64 up

# 夹取时夹持器姿态
int16 grasp_posture

# 夹取前夹持器姿态
int16 pre_grasp_posture


================================================================================
MSG: warehouse/Pose
geometry_msgs/Point position
warehouse/Rotate rotation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: warehouse/Rotate
float64 r
float64 p
float64 y

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['grasp_pos','grasp_approach','grasp_retreat','up','grasp_posture','pre_grasp_posture']
  _slot_types = ['warehouse/Pose','geometry_msgs/Vector3','geometry_msgs/Vector3','float64','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       grasp_pos,grasp_approach,grasp_retreat,up,grasp_posture,pre_grasp_posture

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Grasp, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.grasp_pos is None:
        self.grasp_pos = warehouse.msg.Pose()
      if self.grasp_approach is None:
        self.grasp_approach = geometry_msgs.msg.Vector3()
      if self.grasp_retreat is None:
        self.grasp_retreat = geometry_msgs.msg.Vector3()
      if self.up is None:
        self.up = 0.
      if self.grasp_posture is None:
        self.grasp_posture = 0
      if self.pre_grasp_posture is None:
        self.pre_grasp_posture = 0
    else:
      self.grasp_pos = warehouse.msg.Pose()
      self.grasp_approach = geometry_msgs.msg.Vector3()
      self.grasp_retreat = geometry_msgs.msg.Vector3()
      self.up = 0.
      self.grasp_posture = 0
      self.pre_grasp_posture = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_13d2h().pack(_x.grasp_pos.position.x, _x.grasp_pos.position.y, _x.grasp_pos.position.z, _x.grasp_pos.rotation.r, _x.grasp_pos.rotation.p, _x.grasp_pos.rotation.y, _x.grasp_approach.x, _x.grasp_approach.y, _x.grasp_approach.z, _x.grasp_retreat.x, _x.grasp_retreat.y, _x.grasp_retreat.z, _x.up, _x.grasp_posture, _x.pre_grasp_posture))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.grasp_pos is None:
        self.grasp_pos = warehouse.msg.Pose()
      if self.grasp_approach is None:
        self.grasp_approach = geometry_msgs.msg.Vector3()
      if self.grasp_retreat is None:
        self.grasp_retreat = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 108
      (_x.grasp_pos.position.x, _x.grasp_pos.position.y, _x.grasp_pos.position.z, _x.grasp_pos.rotation.r, _x.grasp_pos.rotation.p, _x.grasp_pos.rotation.y, _x.grasp_approach.x, _x.grasp_approach.y, _x.grasp_approach.z, _x.grasp_retreat.x, _x.grasp_retreat.y, _x.grasp_retreat.z, _x.up, _x.grasp_posture, _x.pre_grasp_posture,) = _get_struct_13d2h().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_13d2h().pack(_x.grasp_pos.position.x, _x.grasp_pos.position.y, _x.grasp_pos.position.z, _x.grasp_pos.rotation.r, _x.grasp_pos.rotation.p, _x.grasp_pos.rotation.y, _x.grasp_approach.x, _x.grasp_approach.y, _x.grasp_approach.z, _x.grasp_retreat.x, _x.grasp_retreat.y, _x.grasp_retreat.z, _x.up, _x.grasp_posture, _x.pre_grasp_posture))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.grasp_pos is None:
        self.grasp_pos = warehouse.msg.Pose()
      if self.grasp_approach is None:
        self.grasp_approach = geometry_msgs.msg.Vector3()
      if self.grasp_retreat is None:
        self.grasp_retreat = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 108
      (_x.grasp_pos.position.x, _x.grasp_pos.position.y, _x.grasp_pos.position.z, _x.grasp_pos.rotation.r, _x.grasp_pos.rotation.p, _x.grasp_pos.rotation.y, _x.grasp_approach.x, _x.grasp_approach.y, _x.grasp_approach.z, _x.grasp_retreat.x, _x.grasp_retreat.y, _x.grasp_retreat.z, _x.up, _x.grasp_posture, _x.pre_grasp_posture,) = _get_struct_13d2h().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_13d2h = None
def _get_struct_13d2h():
    global _struct_13d2h
    if _struct_13d2h is None:
        _struct_13d2h = struct.Struct("<13d2h")
    return _struct_13d2h
