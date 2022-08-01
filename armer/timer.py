"""
Context-based Timer

.. codeauthor:: Gavin Suddreys
"""

import rospy


class Timer:
    """[summary]
    Context-based Timer class that prints the duration of the context
    as well as the frequency.
    """
    def __init__(self, name, enabled=True):
        """[summary]

        :param name: [description]
        :type name: [type]
        :param enabled: [description], defaults to True
        :type enabled: bool, optional
        """
        self.name = name
        self.enabled = enabled
        self.start = 0

    def __enter__(self):
        self.start = rospy.get_time()
        return self

    def __exit__(self, *args):
        if self.enabled:
            dt = rospy.get_time() - self.start
            print('{}: {} ({} hz)'.format(self.name, dt, 1/dt))
