"""autogenerated by genpy from opencv_display/LocaPose.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class LocaPose(genpy.Message):
  _md5sum = "af3859092f247ba2ebdd9f8841635b39"
  _type = "opencv_display/LocaPose"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 t00
float64 t01
float64 t02
float64 t03
float64 t10
float64 t11
float64 t12
float64 t13
float64 t20
float64 t21
float64 t22
float64 t23

"""
  __slots__ = ['t00','t01','t02','t03','t10','t11','t12','t13','t20','t21','t22','t23']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LocaPose, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.t00 is None:
        self.t00 = 0.
      if self.t01 is None:
        self.t01 = 0.
      if self.t02 is None:
        self.t02 = 0.
      if self.t03 is None:
        self.t03 = 0.
      if self.t10 is None:
        self.t10 = 0.
      if self.t11 is None:
        self.t11 = 0.
      if self.t12 is None:
        self.t12 = 0.
      if self.t13 is None:
        self.t13 = 0.
      if self.t20 is None:
        self.t20 = 0.
      if self.t21 is None:
        self.t21 = 0.
      if self.t22 is None:
        self.t22 = 0.
      if self.t23 is None:
        self.t23 = 0.
    else:
      self.t00 = 0.
      self.t01 = 0.
      self.t02 = 0.
      self.t03 = 0.
      self.t10 = 0.
      self.t11 = 0.
      self.t12 = 0.
      self.t13 = 0.
      self.t20 = 0.
      self.t21 = 0.
      self.t22 = 0.
      self.t23 = 0.

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
      buff.write(_struct_12d.pack(_x.t00, _x.t01, _x.t02, _x.t03, _x.t10, _x.t11, _x.t12, _x.t13, _x.t20, _x.t21, _x.t22, _x.t23))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 96
      (_x.t00, _x.t01, _x.t02, _x.t03, _x.t10, _x.t11, _x.t12, _x.t13, _x.t20, _x.t21, _x.t22, _x.t23,) = _struct_12d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_12d.pack(_x.t00, _x.t01, _x.t02, _x.t03, _x.t10, _x.t11, _x.t12, _x.t13, _x.t20, _x.t21, _x.t22, _x.t23))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 96
      (_x.t00, _x.t01, _x.t02, _x.t03, _x.t10, _x.t11, _x.t12, _x.t13, _x.t20, _x.t21, _x.t22, _x.t23,) = _struct_12d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_12d = struct.Struct("<12d")
