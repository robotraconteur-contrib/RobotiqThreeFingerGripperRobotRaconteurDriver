using System;
using System.Collections.Generic;
using System.Text;
using com.robotraconteur.robotics.tool;
using com.robotraconteur.robotics.robot;
using RobotRaconteur;

namespace RobotiqThreeFingerGripperRobotRaconteurDriver
{
    class RobotiqThreeFingerGripper : Tool_default_impl, IDisposable
    {
        string _gripper_ip;
        detail.GripperHost_impl _gripper_host_impl;
        detail.Gripper_impl _gripper_impl;
        public RobotiqThreeFingerGripper(string gripper_ip)
        {
            _gripper_ip = gripper_ip;
            _gripper_host_impl = new detail.GripperHost_impl();
        }

        public void _start_tool()
        {
            _gripper_host_impl.initialize(_gripper_ip);
            _gripper_impl = _gripper_host_impl.get_g(0);
        }

        public override void open()
        {
            _gripper_impl.moveallto(0x00, 0x80, 0x80);
        }

        public override void close()
        {
            _gripper_impl.moveallto(0xFF, 0x80, 0x80);
        }

        public void Dispose()
        {
            _gripper_host_impl?.shutdown();
        }
    }
}
