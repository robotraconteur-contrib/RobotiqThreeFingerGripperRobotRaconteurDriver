using System;
using System.Collections.Generic;
using Mono.Options;
using RobotRaconteur;
using RobotRaconteur.Companion.InfoParser;
using Mono.Unix;
using System.Text.RegularExpressions;

namespace RobotiqThreeFingerGripperRobotRaconteurDriver
{
    class Program
    {

        static int Main(string[] args)
        {



            bool shouldShowHelp = false;
            string tool_info_file = null;
            string gripper_ip = null;
            bool wait_signal = false;

            var options = new OptionSet {
                { "tool-info-file=", n => tool_info_file = n },
                { "gripper-ip=", "the ip address of the gripper", n=> gripper_ip = n },
                {"wait-signal", "wait for POSIX sigint or sigkill to exit", n=> wait_signal = n!=null},
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null }
                };

            List<string> extra;
            try
            {
                // parse the command line
                extra = options.Parse(args);
            }
            catch (OptionException e)
            {
                // output some error message
                Console.Write("RobotiqThreeFingerGripperRobotRaconteurDriver: ");
                Console.WriteLine(e.Message);
                Console.WriteLine("Try `RobotiqThreeFingerGripperRobotRaconteurDriver --help' for more information.");
                return 1;
            }

            if (shouldShowHelp)
            {
                Console.WriteLine("Usage: RobotiqThreeFingerGripperRobotRaconteurDriver [Options+]");
                Console.WriteLine();
                Console.WriteLine("Options:");
                options.WriteOptionDescriptions(Console.Out);
                Console.WriteLine("Also supports standard --robotraconteur- node options");
                return 0;
            }

            if (tool_info_file == null)
            {
                //Console.WriteLine("error: robot-info-file must be specified");
                //return 1;
            }

            if (gripper_ip == null)
            {
                Console.WriteLine("error: gripper-ip must be specified");
                return 1;
            }



            using (var node_setup = new ServerNodeSetup("robotiq_gripper", 58323, args))
            {
                using (var tool = new RobotiqThreeFingerGripper(gripper_ip))
                {
                    tool._start_tool();
                    RobotRaconteurNode.s.RegisterService("tool", "com.robotraconteur.robotics.tool", tool);

                    if (!wait_signal)
                    {
                        Console.WriteLine("Press enter to exit");
                        Console.ReadKey();
                    }
                    else
                    {
                        UnixSignal[] signals = new UnixSignal[]{
                                    new UnixSignal (Mono.Unix.Native.Signum.SIGINT),
                                    new UnixSignal (Mono.Unix.Native.Signum.SIGTERM),
                                };

                        Console.WriteLine("Press Ctrl-C to exit");
                        // block until a SIGINT or SIGTERM signal is generated.
                        int which = UnixSignal.WaitAny(signals, -1);

                        Console.WriteLine("Got a {0} signal, exiting", signals[which].Signum);
                    }
                }

            }

            return 0;


        }
    }
}
