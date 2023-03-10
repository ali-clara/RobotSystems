# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hiwonder_servo_msgs/MultiRawIdPosDur.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hiwonder_servo_msgs.msg

class MultiRawIdPosDur(genpy.Message):
  _md5sum = "219f46020f0bb7e7755eb26cd4b971ed"
  _type = "hiwonder_servo_msgs/MultiRawIdPosDur"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """RawIdPosDur[] id_pos_dur_list
================================================================================
MSG: hiwonder_servo_msgs/RawIdPosDur
uint8 id
uint16 position
uint16 duration
"""
  __slots__ = ['id_pos_dur_list']
  _slot_types = ['hiwonder_servo_msgs/RawIdPosDur[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id_pos_dur_list

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MultiRawIdPosDur, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id_pos_dur_list is None:
        self.id_pos_dur_list = []
    else:
      self.id_pos_dur_list = []

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
      length = len(self.id_pos_dur_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.id_pos_dur_list:
        _x = val1
        buff.write(_get_struct_B2H().pack(_x.id, _x.position, _x.duration))
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
      if self.id_pos_dur_list is None:
        self.id_pos_dur_list = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.id_pos_dur_list = []
      for i in range(0, length):
        val1 = hiwonder_servo_msgs.msg.RawIdPosDur()
        _x = val1
        start = end
        end += 5
        (_x.id, _x.position, _x.duration,) = _get_struct_B2H().unpack(str[start:end])
        self.id_pos_dur_list.append(val1)
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
      length = len(self.id_pos_dur_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.id_pos_dur_list:
        _x = val1
        buff.write(_get_struct_B2H().pack(_x.id, _x.position, _x.duration))
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
      if self.id_pos_dur_list is None:
        self.id_pos_dur_list = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.id_pos_dur_list = []
      for i in range(0, length):
        val1 = hiwonder_servo_msgs.msg.RawIdPosDur()
        _x = val1
        start = end
        end += 5
        (_x.id, _x.position, _x.duration,) = _get_struct_B2H().unpack(str[start:end])
        self.id_pos_dur_list.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B2H = None
def _get_struct_B2H():
    global _struct_B2H
    if _struct_B2H is None:
        _struct_B2H = struct.Struct("<B2H")
    return _struct_B2H
