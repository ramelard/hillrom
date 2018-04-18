from ctypes import *


# Use like this:
#   carr = (c_int*3)(*arr)
#   res = mydll.emxCreateND_real_T(3,carr)
#   emxarray_ptr = cast(res,POINTER(emxarray_real_T))
#   emxarray = emxarray_ptr.contents
class emxarray_real_T(Structure):
  """Matches emxArray_real_T in extract_vitals_types.h.
  To create instances, see functions in extract_vitals_emxAPI.h
  """
  _fields_ = [('data', POINTER(c_double)),
              ('size', POINTER(c_int)),  # array of size of each dimension
              ('allocatedSize', c_int),
              ('numDimensions', c_int),
              ('canFreeData', c_ubyte)]  # c_ubyte to match boolean_T
