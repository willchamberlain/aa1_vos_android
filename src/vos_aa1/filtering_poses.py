import numpy

# sys.path.append('/mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/src/vos_aa1/')
# from filtering_poses import filter

# del sys.modules["filtering_poses"]
# del filtering_poses


def filter(y_nump_array, new_measurement):
  if y_nump_array.size < 20:
    y_nump_array = numpy.append(y_nump_array, new_measurement)
    return {'ready':False, 'filtered_array':y_nump_array, 'unfiltered_array':y_nump_array, 'average':9000.00}


  average_ = numpy.average(y_nump_array)

  upper_bound = average_ * 1.5

  lower_bound = average_ * 0.5

  y_nump_array = y_nump_array[1:y_nump_array.size]

  y_nump_array = numpy.append(y_nump_array, new_measurement)

  y_nump_array_upper = y_nump_array[y_nump_array <= upper_bound]

  y_nump_array_filtered = y_nump_array_upper[y_nump_array_upper >= lower_bound ]

  y_nump_array_filtered

  numpy.average(y_nump_array)

  average_ = numpy.average(y_nump_array_filtered)

  return {'ready':True, 'filtered_array':y_nump_array_filtered, 'unfiltered_array':y_nump_array, 'average':average_}



def unload_filtering_poses():
    del sys.modules["filtering_poses"]
    del filtering_poses

def reload_filtering_poses():
    unload_filtering_poses()
    from filtering_poses import filter
