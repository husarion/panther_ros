#!/usr/bin/python3

import os
import paramiko
import socket

import rospy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest

class ShutdownNode:
    def __init__(self, name) -> None:

        rospy.init_node(name, anonymous=False)

        self._shutdown_timeout = rospy.get_param('~shutdown_timeout', 15.0)
        
        self._ip = rospy.get_param('~self_ip', '127.0.0.1')
        self._username = rospy.get_param('~self_username', 'husarion')
        
        dif = rospy.get_param('~default_identity_file', '~/.ssh/id_rsa')
        self._default_identity_file = os.path.expanduser(dif)
        if rospy.has_param('~self_identity_file'):
            sif = rospy.get_param('~self_identity_file')
            self._identity_file = os.path.expanduser(sif)
        else:
            self._identity_file = self._default_identity_file
            
        self._hosts = rospy.get_param('~hosts', [])
        for host in self._hosts:
            # check if all heys are provided
            if {'ip', 'username'} != set(host.keys()):
                rospy.logerr(f'[{rospy.get_name()}] Missing info for remote host!')
                raise Exception
            if 'identity_file' not in host.keys():
                host['identity_file'] = self._default_identity_file
            else:
                host['identity_file'] = os.path.expanduser(host['identity_file'])
                
            if not os.path.exists(host['identity_file']):
                rospy.logerr(f'[{rospy.get_name()}]'
                    f' Can\'t find provided identity file for host {host["ip"]}!'
                    f' Path \'{host["identity_file"]}\' doesn\'t exist')
                raise Exception
            
            if 'cmd' not in host.keys():
                host['cmd'] = 'sudo shutdown now'
        
        # -------------------------------
        #   Services
        # -------------------------------

        self._shutdown_srv = rospy.Service(
            'shutdown', SetBool, self._shutdown_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')
        
    def _check_ip(self, host):
      return os.system('ping -c 1 -w 1 ' + host + ' > /dev/null') == 0
    
    def _shutdown_cb(self) -> None:          
        rospy.logwarn(f'[{rospy.get_name()}] Soft shutdown initialized.')
        # create new list of computers that confirmed shutdown procedure
        hosts_to_check = [h for h in self._hosts
                          if self._request_shutdown(h['ip'], h['identity_file'], h['username'], h['cmd'])]
        
        start_time = rospy.get_time()
        if len(hosts_to_check) > 0:
            while rospy.get_time() - start_time < self._shutdown_timeout:
                # reduce list to all computers that are still available
                hosts_to_check = [h for h in hosts_to_check if self._check_ip(h['ip'])]
                if len(hosts_to_check) == 0:
                    rospy.loginfo(f'[{rospy.get_name()}] All computes shut down gracefully.')
                    break
            else:
                rospy.loginfo(f'[{rospy.get_name()}] '
                    'Shutdown timeout reached. Cutting out power from computers.')
            
        # ensure all computers did full shutdown
        rospy.loginfo(f'[{rospy.get_name()}] Shutting down itself.')
        self._request_shutdown(self._ip, self._identity_file, self._username, 'sudo shutdown now')
        
    def _request_shutdown(self, ip, identity_file, username, cmd) -> bool:
        # shutdown only if host available
        if self._check_ip(ip):
            try:
                pkey = paramiko.RSAKey.from_private_key_file(identity_file)
                client = paramiko.SSHClient()
                policy = paramiko.AutoAddPolicy()
                client.set_missing_host_key_policy(policy)
                client.connect(ip, username=username, pkey=pkey, timeout=0.5)
                rospy.loginfo(f'[{rospy.get_name()}] Shutting down device at {ip}')
                try:
                    client.exec_command(cmd, timeout=0.5)
                except socket.timeout:
                    # some systems do not close SSH connection on shut down
                    # this will handle the timeout to allow shutting other devices
                    pass
                client.close()
                return True
            except Exception as e:
                rospy.logerr(f'[{rospy.get_name()}] Can\'t SSH to device at {ip}!')
                return False
        else:
            rospy.loginfo(f'[{rospy.get_name()}] Device at {ip} not available.')
            return False


def main():
    shutdown_node = ShutdownNode('shutdown_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
