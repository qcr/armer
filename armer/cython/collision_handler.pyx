from libc.math cimport log
from libcpp.list cimport list as cpplist
from libcpp.string cimport string
cimport cython

import numpy as np
cimport numpy as cnp
cnp.import_array()
DTYPE = np.float64
ctypedef cnp.float64_t DTYPE_t

cdef extern from "<vector>" namespace "std":
  cdef cppclass vector[T]:
    void push_back(T&) nogil
    size_t size()
    T& operator[](size_t)

@cython.wraparound(False)
cpdef dict closest_dist_query(str sliced_link_name, 
                              list col_link_names, 
                              int col_link_name_len, 
                              dict col_dict,
                              dict overlap_dict):

  cdef int col_link_idx
  cdef cnp.ndarray dist
  cdef list c_obj_list
  cdef list slice_obj_list
  # cdef list out_list
  # cdef vector[float] dist_list
  # cdef vector[vector[DTYPE_t]] out_list
  cdef dict out_dict = {}
  # cdef float temp
  cdef str link

  # Method definition
  # push_back_main = out_list.push_back
  # push_back = dist_list.push_back
  for col_link_idx in range(col_link_name_len):
    # Only add to output list if within the following criteria
    link = col_link_names[col_link_idx]
    c_obj_list = col_dict[link]
    slice_obj_list = col_dict[sliced_link_name]
    if c_obj_list == [] or slice_obj_list == []:
      continue

    if link in overlap_dict[sliced_link_name]:
      continue

    # if col_link_names[col_link_idx] not in overlap_dict[sliced_link_name]:
      # Calculate the closest point distance between first shape available
    _, _, dist = slice_obj_list[0].closest_point(c_obj_list[0])
    # out_dict[col_link_names[col_link_idx]] = dist[2]
    # out_list.push_back(dist[2])
    out_dict[link] = dist

  return out_dict

# Ignore negative indice checking (i.e., a[-1])
@cython.wraparound(False)
cpdef int col_check(list sliced_links, list check_links, int len_links, dict global_dict):

  cdef int s_idx
  cdef int e_idx
  cdef int eval_link_list_len
  cdef int col_list_len
  cdef int chk_list_len
  cdef list t_link_names
  cdef list col_list
  cdef list c_check_list

  for s_idx in range(len_links):
    # Get the current current slice link's collision objects to check against
    col_list = list(sliced_links[s_idx].collision)
    col_list_len = len(col_list)

    # Get target links as list
    t_link_names = check_links[s_idx]
    eval_link_list_len = len(t_link_names)
    for e_idx in range(eval_link_list_len):
      # Extract the evaluation robot link's collision object list for checking
      c_check_list = global_dict[t_link_names[e_idx]]
      chk_list_len = len(c_check_list)

      for chk_idx in range(chk_list_len):
        obj = c_check_list[chk_idx]

        for col_idx in range(col_list_len):
          t_obj = col_list[col_idx]

          # Get the distance for comparison
          # print(f"obj: {t_obj}")
          d, _, _ = t_obj.closest_point(obj)
          # print(f"d is: {d}")
          if d is not None and d <= 0:
            # Returns the link idx in collision
            return s_idx

  # Got here without issue
  return -1

# Ignore negative indice checking (i.e., a[-1])
@cython.wraparound(False)
cpdef int global_check(str robot_name, 
                      list robot_names, 
                      int len_robots, 
                      list robot_links, 
                      int len_links,
                      dict global_dict, 
                      dict overlap_dict,
                      list check_links):
  cdef list c_ignore_list
  cdef list c_check_list
  cdef list col_list
  
  cdef str r_name
  cdef str l_name
  cdef str t_l_name
  
  cdef int r_idx
  cdef int col_idx
  cdef int chk_idx
  cdef int l_idx

  cdef int col_list_len
  cdef int chk_list_len

  # Iterate through all instantiated robots
  # - there may be more than one robot, so loop through each
  # - for the current robot, we want to evaluate one of the sliced links over the current dictionary of links
  #   - example, panda robot slice link 1 is panda_hand  
  for r_idx in range(len_robots):
    # Get the robot name for evaluation
    r_name = robot_names[r_idx]
    # DEBUGGING
    # print(f"NEW robot: {robot_names[idx]}" )

    # Loop though the global dictionary of links for robot in evaluation
    for l_name in global_dict[r_name].copy():
      # DEBUGGING
      # print(f"r_name: {r_name} | l_name: {l_name}")

      # Extract the evaluation robot link's collision object list for checking
      c_check_list = global_dict[r_name][l_name]

      # Handle self checking and populate overlap links for ignoring
      if robot_name == r_name and l_name in overlap_dict:
        c_ignore_list = overlap_dict[l_name]
      else:
        c_ignore_list = []

      # Local checking of evaluation robot's link with robot's target links
      # NOTE: target links are defined as those links of particular note for checking
      #       speed up achived by limiting this list 
      #       (search space from end-effector down to base_link, sliced as needed)
      for l_idx in range(len_links):
        link = robot_links[l_idx]
        t_l_name = link.name
        # DEBUGGING
        # print(f"checking {t_l_name}")

        # TEST (break out if not one of the defined target links)
        if l_name not in check_links[l_idx]:
          break

        # Skip to next target link if it is to be ignored
        if t_l_name in c_ignore_list:
          continue

        # Skip to next target link if it is the same link as current being checked
        if t_l_name == l_name:
          continue

        # Evaluate current link's collision shapes
        chk_list_len = len(c_check_list)
        for chk_idx in range(chk_list_len):
          obj = c_check_list[chk_idx]
          col_list = list(link.collision)
          col_list_len = len(col_list)
          for col_idx in range(col_list_len):
            t_obj = col_list[col_idx]

            # Get the distance for comparison
            # print(f"obj: {t_obj}")
            d, _, _ = t_obj.closest_point(obj)
            # print(f"d is: {d}")
            if d is not None and d <= 0:
              # Returns the link idx in collision
              return l_idx

  # Got here without issue
  return -1

cpdef int test (int val1, int val2):
  return val1 + val2