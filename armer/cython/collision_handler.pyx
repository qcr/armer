cimport cython

# Ignore negative indice checking (i.e., a[-1])
@cython.wraparound(False)
cpdef int global_check(str robot_name, 
                      list robot_names, 
                      int len_robots, 
                      list robot_links, 
                      int len_links,
                      dict global_dict, 
                      dict overlap_dict):
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
  for r_idx in range(len_robots):
    # Get the robot name for evaluation
    r_name = robot_names[r_idx]
    # DEBUGGING
    # print(f"NEW robot: {robot_names[idx]}" )

    # Loop though the global dictionary of links for robot in evaluation
    for l_name in global_dict[r_name]:
      # DEBUGGING
      # print(f"r_name: {r_name} | l_name: {l_name}")

      # Extract the evaluation robot link's collision object list for checking
      c_check_list = global_dict[r_name][l_name]
      # Handle self checking and populate overlap links for ignoring
      if robot_name == r_name and l_name in overlap_dict:
        c_ignore_list = overlap_dict[l_name]
      else:
        c_ignore_list = list()

      # Local checking of evaluation robot's link with robot's target links
      # NOTE: target links are defined as those links of particular note for checking
      #       speed up achived by limiting this list 
      #       (search space from end-effector down to base_link, sliced as needed)
      for l_idx in range(len_links):
        link = robot_links[l_idx]
        t_l_name = link.name
        # DEBUGGING
        # print(f"checking {t_l_name}")

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