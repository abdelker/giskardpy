import rospy
from inspect import currentframe, getframeinfo
from giskardpy import identifier

def generate_debug_msg(msg):
    node_name = rospy.get_name()
    frameinfo = getframeinfo(currentframe().f_back.f_back)
    file_info = frameinfo.filename.split('/')[-1] + ' line ' + str(frameinfo.lineno)
    new_msg = ('\nnode: ' + node_name + '\n' +
               'file: ' + file_info + '\n' +
               'message: ' + msg + '\n')
    return new_msg

def generate_msg(msg):
    node_name = rospy.get_name()
    new_msg = '[' + node_name + ']' + ': ' + msg
    return new_msg



def debug():
    if debug.param == None:
        l = identifier.debug
        param_name = '/giskard/' + '/'.join(s for s in l[1:])
        debug.param = rospy.get_param(param_name)
        return debug.param
    else:
        return debug.param

debug.param = None

def logdebug(msg):
    final_msg = generate_debug_msg(msg)
    rospy.logdebug(final_msg)

def loginfo(msg):
    if (debug()):
        final_msg = generate_debug_msg(msg)
    else:
        final_msg = generate_msg(msg)
    rospy.loginfo(final_msg)

def logwarn(msg):
    if (debug()):
        final_msg = generate_debug_msg(msg)
    else:
        final_msg = generate_msg(msg)
    rospy.logwarn(final_msg)

def logerr(msg):
    if (debug()):
        final_msg = generate_debug_msg(msg)
    else:
        final_msg = generate_msg(msg)
    rospy.logerr(final_msg)

def logfatal(msg):
    if (debug()):
        final_msg = generate_debug_msg(msg)
    else:
        final_msg = generate_msg(msg)
    rospy.logfatal(final_msg)

