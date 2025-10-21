#!/usr/bin/python3
# encoding: utf-8
# @data:2022/12/05
# @author:aiden
# 多点导航
import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped

goal_count = 0
try_again = True
goal_reached_count = 0
haved_clicked = False
add_more_point = False
enable_navigation = True
markerArray = MarkerArray()

# move_base状态回调(move_base status callback)
def status_callback(msg):
    global try_again, goal_reached_count, add_more_point, enable_navigation

    if haved_clicked:  # 防止通过其他方式发布点的回调进来(prevent callbacks from publishing points by other means from coming in)
        try:
            if msg.status.status == 3 or msg.status.status == 4:  # 如果是正常到达目标点(if arrive at the goal normally)
                if msg.status.status == 3:
                    if goal_reached_count < goal_count:
                        goal_reached_count += 1  # 发送下一个点(send next point)
                        print('******number %s goal reached******' % goal_reached_count)
                if goal_reached_count < goal_count:  # 如果添加的点还没走完继续发送下一个点(if robot doesn't reach the added point, continue sending next point)
                    if enable_navigation:
                        rospy.sleep(0.1)
                        pose = PoseStamped()
                        pose.header.frame_id = map_frame
                        pose.header.stamp = rospy.Time.now()
                        pose.pose.position.x = markerArray.markers[goal_reached_count].pose.position.x
                        pose.pose.position.y = markerArray.markers[goal_reached_count].pose.position.y
                        pose.pose.orientation.z = 1
                        goal_pub.publish(pose)
                        print('******send number %s goal******' % (goal_reached_count + 1))
                elif goal_reached_count == goal_count:  # 所有点到到达了(reach all the points)
                    print('******all points reach******')
                    add_more_point = True
                try_again = True
            else:  # 没有正常到达目标则再次尝试，(retry if robot doesn't reach the goal normally)
                print('******Goal cannot reached has some error :', msg.status.status, ' try again!!!!')
                if try_again:  # 尝试一次失败后不再尝试，直接发送下一个点(if the attempt ends in failure, stop and directly send next point)
                    pose = PoseStamped()
                    pose.header.frame_id = map_frame
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = markerArray.markers[goal_reached_count].pose.position.x
                    pose.pose.position.y = markerArray.markers[goal_reached_count].pose.position.y
                    pose.pose.orientation.z = 1
                    goal_pub.publish(pose)
                    try_again = False
                elif goal_reached_count < goal_count:
                    goal_reached_count += 1
                    pose = PoseStamped()
                    pose.header.frame_id = map_frame
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = markerArray.markers[goal_reached_count].pose.position.x
                    pose.pose.position.y = markerArray.markers[goal_reached_count].pose.position.y
                    pose.pose.orientation.z = 1
                    goal_pub.publish(pose)
        except BaseException as e:
            print(e)


def click_callback(msg):
    global add_more_point, markerArray
    global goal_count, haved_clicked

    goal_count += 1
    print('******add number %s goal******' % goal_count)

    # 用数字标记来显示点(mark the point with number to display)
    marker = Marker()
    marker.header.frame_id = map_frame
    
    #marker.type = marker.MESH_RESOURCE
    #marker.mesh_resource = "package://rviz_plugin/media/flag.dae"
    marker.type = marker.TEXT_VIEW_FACING  # 类型为text(type is text)
    marker.action = marker.ADD
    # 大小(size)
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    # 颜色(color)
    color = list(np.random.choice(range(256), size=3))
    marker.color.a = 1
    marker.color.r = color[0]/255.0
    marker.color.g = color[1]/255.0
    marker.color.b = color[2]/255.0
    # marker.lifetime = rospy.Duration(20)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
    # 位置姿态
    marker.pose.position.x = msg.point.x
    marker.pose.position.y = msg.point.y
    marker.pose.position.z = msg.point.z
    marker.pose.orientation.w = 1
    # 显示内容(display content)
    marker.text = str(goal_count)

    # 添加到列表以保持显示(add to the list to maintain display)
    markerArray.markers.append(marker)
    marker_id = 0
    for m in markerArray.markers:
        m.id = marker_id
        marker_id += 1
    mark_pub.publish(markerArray)

    if enable_navigation:
        if goal_count == 1:  # 添加第一个点时，立刻发送到move_base(send to move_base immediately when adding the first point)
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = msg.point.x
            pose.pose.position.y = msg.point.y
            pose.pose.orientation.z = 1
            goal_pub.publish(pose)
            print('******send number %s goal******' % goal_count)
        elif add_more_point:  # 机器人已到达所有点，此时添加点要手动触发回调(robot arrives at all points. At this time, At this point, add a point to manually trigger the callback)
            add_more_point = False
            move = MoveBaseActionResult()
            move.status.status = 4
            move.header.stamp = rospy.Time.now()
            print('******add more point******')
            goal_status_pub.publish(move)

    haved_clicked = True

def start_navigation_srv_callback(msg):
    global enable_navigation

    print('******start navigation******')

    enable_navigation = True

    move = MoveBaseActionResult()
    move.status.status = 4
    move.header.stamp = rospy.Time.now()
    goal_status_pub.publish(move)

    return EmptyResponse()

def stop_navigation_srv_callback(msg):
    global enable_navigation

    print('******stop navigation******')

    enable_navigation = False

    move = MoveBaseActionResult()
    move.status.status = 4
    move.header.stamp = rospy.Time.now()

    goal_status_pub.publish(move)

    return EmptyResponse()

def clear_goals_srv_callback(msg):
    global markerArray, goal_count

    print('******clear goals******')
    goal_count = 0
    markerArray = MarkerArray()
    marker_Array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = map_frame
    marker.action = Marker.DELETEALL
    marker_Array.markers.append(marker)

    mark_pub.publish(marker_Array)
    
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('publish_point_node')

    map_frame = rospy.get_param('~map_frame', 'map')
    enable_navigation = rospy.get_param('~enable_navigation', True)
    clicked_point = rospy.get_param('~clicked_point', '/clocked_point')
    click_sub = rospy.Subscriber(clicked_point, PointStamped, click_callback)

    start_navigation_srv = rospy.get_param('~start_navigation', '/start_navigation')
    stop_navigation_srv = rospy.get_param('~stop_navigation', '/stop_navigation')
    clear_goals_srv = rospy.get_param('~clear_goals', '/clear_goals')

    rospy.Service(start_navigation_srv, Empty, start_navigation_srv_callback)
    rospy.Service(stop_navigation_srv, Empty, stop_navigation_srv_callback)
    rospy.Service(clear_goals_srv, Empty, clear_goals_srv_callback)

    mark_pub = rospy.Publisher('path_point', MarkerArray, queue_size=100)
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

    move_base_result = rospy.get_param('~move_base_result', '/move_base/result')
    goal_status_sub = rospy.Subscriber(move_base_result, MoveBaseActionResult, status_callback)
    goal_status_pub = rospy.Publisher(move_base_result, MoveBaseActionResult, queue_size=1)

    rospy.spin()
