using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace RobotiqThreeFingerGripperRobotRaconteurDriver.detail
{
    
    // Old software taken from Rensselaer Polytechnic Institute RobotiqGripperInterface project
    class GripperHost_impl
    {
        Gripper_impl[] grippers;
        Thread t;
        Thread t2;
        public void initialize(string gripper_ip)
        {
            // Set up two grippers and two F/T sensors
            grippers = new Gripper_impl[1];
                
            grippers[0] = new Gripper_impl(gripper_ip);                         

            // Daemon thread start
            t = new Thread(new ThreadStart(daemon));
            t.Start();

            // Background thread start
            t2 = new Thread(new ThreadStart(backgroundThread));
            t2.Start();
        }

        public void shutdown()
        {
            t2.Abort();
            t.Abort();
            // Give it a moment to finish all lagging operations
            Thread.Sleep(100);
            grippers[0].disconnect();
            grippers[1].disconnect();
        }

        // Return a gripper reference
        public Gripper_impl get_g(int ind)
        {
            if (ind == 0 || ind == 1)
            {
                return grippers[ind];
            }
            else
                throw new Exception("Bad index");
        }

        // Return a sensor reference
            
        // Daemon runs in a separate thread. It checks flags which are set by the
        // interface functions (move, changemode, etc) and acts accordingly.
        public void daemon()
        {
            while (true)
            {
                //Take care of output commands first
                for (int i = 0; i < 2; i++)
                {
                    if (grippers[i].found)
                    {
                        //Check stop flag and stop the gripper if set
                        if (grippers[i].stop_f)
                        {
                            grippers[i].startstop();
                            grippers[i].stop_f = false;
                            grippers[i].move_inprocess_f = false;
                            grippers[i].modechange_inprocess_f = false;
                        }
                        //Check move flag and start a moving command if set
                        if (grippers[i].move_f)
                        {
                            grippers[i].startmove();
                            grippers[i].move_f = false;
                            grippers[i].move_inprocess_f = true;
                            grippers[i].modechange_inprocess_f = false;
                            grippers[i].completed = false;
                            //grippers[i].completed_timer.Start();
                        }
                        //Check changemode flag and start a change mode command if set
                        if (grippers[i].changemode_f)
                        {
                            grippers[i].startchangemode();
                            grippers[i].changemode_f = false;
                            grippers[i].move_inprocess_f = false;
                            grippers[i].modechange_inprocess_f = true;
                            grippers[i].completed = false;
                            //grippers[i].completed_timer.Start();
                        }
                    }
                }
                // Now take care of input commands
                for (int i = 0; i < 2; i++)
                {
                    if ((grippers[i].sendback_f || grippers[i].move_inprocess_f ||
                            grippers[i].modechange_inprocess_f) && grippers[i].found)
                    {
                        Thread.Sleep(5); // Gripper runs at 200Hz. It is necessary to wait 5ms
                                            // in between queries.
                        grippers[i].readRegisters(); // Get the newest data (all register values)
                        if (grippers[i].sendback_f)
                        {
                            // Other thread will be blocked by the sendback flag while data is
                            // being requested. Now unblock that thread.
                            grippers[i].sendback_f = false;
                        }
                        if (grippers[i].move_inprocess_f)
                        {
                            // Check if gripper is finished moving
                            if ((byte)(grippers[i].received_data[0] & 0xC0) != 0x00 &&
                                    ((byte)(grippers[i].received_data[0] & 0x08) >> 3) == 1)
                            {
                                Console.WriteLine("Move Completed!");
                                grippers[i].move_inprocess_f = false;
                                grippers[i].completed = true;
                                grippers[i].call_event(); //Call the movecomplete() event
                            }

                        }
                        if (grippers[i].modechange_inprocess_f)
                        {
                            // Check if gripper is finished moving
                            if (((byte)(grippers[i].received_data[0] & 0x30) >> 6) != 0 &&
                                    (byte)(grippers[i].received_data[0] & 0x08) == 0x08)
                            {
                                Console.WriteLine("Move Completed!");
                                grippers[i].modechange_inprocess_f = false;
                                grippers[i].completed = true;
                                grippers[i].call_event(); //Call the movecomplete() event
                            }

                        }
                    }
                }
            }
        }

        // This is a background thread which just periodically reads in the data from the gripper registers.
        // This ensures the devices won't go to sleep, and also updates the necessary data to be read.
        public void backgroundThread()
        {
            long loopStartTime;
            SpinWait spinner = new SpinWait();
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            loopStartTime = stopwatch.ElapsedMilliseconds;
            while (true)
            {
                long loopTime = stopwatch.ElapsedMilliseconds;
                    
                // Wait 5 milliseconds between updates
                if ((loopTime - loopStartTime)  > 5) // FIXME BACK TO 5
                {
                    for (int i = 0; i < 2; i++)
                    {
                        if (grippers[i].found)
                        {
                            grippers[i].updateData();
                        }
                    }                        
                    loopStartTime = stopwatch.ElapsedMilliseconds;
                }
                else
                {
                    spinner.SpinOnce();
                }
            }
        }

            

    }

    class Gripper_impl
    {
        public const Byte Gripper_Basic = 0x00;
        public const Byte Gripper_Wide = 0x02;
        public const Byte Gripper_Pinch = 0x01;
        public const Byte Gripper_Scissor = 0x03;


        private ModBus link;
        private Random rand;

        public bool move_f;
        public bool changemode_f;
        public bool stop_f;
        public volatile bool sendback_f;
        public bool move_inprocess_f;
        public bool modechange_inprocess_f;

        private byte[] des_pos;
        private byte[] des_speed;
        private byte[] des_force;
        private byte mode;
        public bool found;

        public byte[] received_data;

        private GripperData d;

        //public System.Timers.Timer completed_timer;
        public bool completed;

        public Gripper_impl(string address)
        {
            found = false;
            link = new ModBus(address); // Set up a new TCP connection under MODBUS.
            found = link.found;
            rand = new Random();

            move_f = false;
            changemode_f = false;
            sendback_f = false;
            move_inprocess_f = false;
            modechange_inprocess_f = false;

            received_data = new byte[16];
            des_pos = new byte[3];
            des_speed = new byte[3];
            des_force = new byte[3];

            d = new GripperData();
            d.currents = new byte[4];
            d.positions = new byte[4];

            // The check that the move is completed (checking two bits in register 0)
            // is not valid until a certain time after the move command was issued. 
            // This was experimentally determined to be no less than ~100ms.
            //completed_timer = new System.Timers.Timer();
            //completed_timer.Elapsed += new ElapsedEventHandler(completed_timer_reset);
            //completed_timer.Interval = 100; //100ms
            //completed = false;

            if (found)
                initialize();
        }

        // Set the completed flag to let the daemon know that the status check is now valid
        //public void completed_timer_reset(object source, ElapsedEventArgs e)
        //{
        //    completed_timer.Stop();
        //    completed = true;
        //}
        public void connect(string address)
        {
            link.connect(address);
        }

        public void disconnect()
        {
            if (found)
                link.disconnect();
        }


        public void initialize()
        {
            // Let gripper perform range-of-motion check if it already hasn't.
            // Set other registers to neutral values
            Byte[] data = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x00 };

            readRegisters();

            if ((byte)(received_data[0] & 0x01) != 1) //If gripper isn't already initialized
            {
                //Initialize gripper
                System.Console.WriteLine("Activating Gripper...");
                writeRegisters(data);
                modechange_inprocess_f = true;
                do
                {                                //Poll until mode change is successful
                    Thread.Sleep(200);           //Poll 5 times a second
                    readRegisters();
                } while ((byte)(received_data[0] & 0x01) != 0x01 || (byte)(received_data[0] & 0x30) != 0x30);
                System.Console.WriteLine("Activated!");
            }
            else
            {
                System.Console.WriteLine("Previously Activated.  Sending command to go to home position..");
                this.moveallto(0, 255, 255);
            }

        }
        // TODO: Get this event handler working
        public event Action movecomplete;

        public void call_event()
        {
            //movecomplete();
        }

        public void changemode(byte mode)
        {
            if (mode == this.mode)
                return;
            this.des_speed[0] = 255; //Maximum speed
            this.des_speed[1] = 255; //Maximum speed
            this.des_speed[2] = 255; //Maximum speed
            this.des_force[0] = 255; //Maximum force
            this.des_force[1] = 255; //Maximum force
            this.des_force[2] = 255; //Maximum force
            if (!(mode >= 0 && mode <= 3))
                throw new Exception("Invalid mode");
            this.mode = mode;
            this.changemode_f = true;
        }

        public void startchangemode()
        {
            Byte[] data = {
                            (byte)(0x01 | (mode << 1)),
                            0x04,
                            0x00,
                            0x00,
                            des_speed[0],
                            des_force[0]
                        };
            writeRegisters(data);
        }

        public void moveto(byte[] position, byte[] speed, byte[] force)
        {
            Array.Copy(position, 0, this.des_pos, 0, 3);
            Array.Copy(speed, 0, this.des_speed, 0, 3);
            Array.Copy(force, 0, this.des_force, 0, 3);
            this.move_f = true;

        }

        public void moveallto(byte position, byte speed, byte force)
        {
            this.des_pos[0] = position;
            this.des_pos[1] = position;
            this.des_pos[2] = position;
            this.des_speed[0] = speed;
            this.des_speed[1] = speed;
            this.des_speed[2] = speed;
            this.des_force[0] = force;
            this.des_force[1] = force;
            this.des_force[2] = force;
            this.move_f = true;
        }

        public void startmove()
        {
            Byte[] data = {
                        (byte)(0x01 | (mode << 1) | 0x08),  // Action Request
                        0x04,                               // Gripper Options
                        0x00,                               // Gripper Options 2 (empty)
                        des_pos[0],                         // Finger A position request
                        des_speed[0],                       // Finger A speed
                        des_force[0],                       // Finger A force
                        des_pos[1],                         // Finger B position request
                        des_speed[1],                       // Finger B speed
                        des_force[1],                       // Finger B force
                        des_pos[2],                         // Finger C position request
                        des_speed[2],                       // Finger C speed
                        des_force[2]                        // Finger C force
                        };
            writeRegisters(data);
        }

        public void stop()
        {
            this.stop_f = true;
        }

        public void startstop()     // TODO: UPDATE THIS
        {
            Byte[] data = {
                        (byte)(0x01 | (mode << 1) | 0x08),
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00
                        };
            writeRegisters(data);
        }

        public GripperData data
        {
            get
            {
                return d;
            }
            set { }
        }

        public void updateData()
        {
            if (found)
            {
                d.found = 1;

                sendback_f = true;
                // Blocks here until daemon clears flag, indicating new data
                // to be read
                while (sendback_f) { }


                for (int i = 0; i < 4; i++)
                {
                    d.positions[i] = received_data[4 + i * 3];
                    d.currents[i] = received_data[5 + i * 3]; // *Note this is 0.1 * current [mA]
                }

                if (completed)
                    d.completed = 1;
                else
                    d.completed = 0;
                d.mode = (byte)((received_data[0] & 0x06) >> 1);

                // Only really care about contact while closing.
                d.a_detect = (byte)(received_data[1] & 0x01);
                d.b_detect = (byte)((received_data[1] & 0x04) >> 2);
                d.c_detect = (byte)((received_data[1] & 0x10) >> 4);
                d.scissors_detect = (byte)((received_data[1] & 0x40) >> 6);
            }
            else
                d.found = 0;
        }

        public void readRegisters()
        {
            //Read in all status registers and then pick out the data we need
            Byte[] transId = BitConverter.GetBytes(rand.Next(0xffff));
            Byte[] received = null;
            Byte[] command =
            {
                    transId[0],   // Transaction Identifier MSB
                    transId[1],   // Transaction Identifier LSB
                    0x00,         // Protocol ID MSB (=0 for ModBus)
                    0x00,         // Protocol ID LSB (=0 for ModBus)
                    0x00,         // Packet Length MSB
                    0x06,         // Packet Length LSB 
                    0x02,         // UnitID (any value will work, 02 used in their example)
                    0x04,         // Read Function Code (0x04)
                    0x00,         // First Register Address MSB
                    0x00,         // First Register Address LSB
                    0x00,         // Number of data words MSB
                    0x08          // Number of data words LSB
        };


            while (!link.sendCommand(command, ref received))
            {
                System.Console.WriteLine("Read failed");      // TODO: Handle this better
                Thread.Sleep(5);
            }
            //System.Console.WriteLine("Read In: " + (received.Length).ToString() + " Bytes");
            Array.Copy(received, 9, received_data, 0, 16);   // First 9 bytes received are just protocol
        }


        public void writeRegisters(Byte[] data)
        {

            Byte[] command = new Byte[13 + data.Length];
            Byte[] transId = BitConverter.GetBytes(rand.Next(0xffff));
            Byte[] received = null;
            command[0] = transId[0];                                // Transaction Identifier MSB
            command[1] = transId[1];                                // Transaction Identifier LSB
            command[2] = 0x00;                                      // Protocol ID MSB (=0 for ModBus)
            command[3] = 0x00;                                      // Protocol ID LSB (=0 for ModBus)
            command[4] = 0x00;                                      // Packet Length MSB
            command[5] = (byte)(7 + data.Length);                   // Packet Length LSB
            command[6] = 0x02;                                      // UnitID (any value will work, 02 used in their example)
            command[7] = 0x10;                                      // Write Function Code (16d = 0x10)
            command[8] = 0x00;                                      // First Register Address MSB 03
            command[9] = 0x00;                                      // First Register Address LSB E8
            command[10] = 0x00;                                     // Number of data words MSB
            command[11] = (byte)Math.Ceiling(data.Length / 2.0);    // Number of data words LSB
            command[12] = (byte)data.Length;                        // Number of data bytes
            Array.Copy(data, 0, command, 13, data.Length);
            if (!link.sendCommand(command, ref received))
            {
                Console.WriteLine("Write did not receive response - Assumed Write Failure");
                //throw new Exception("writeRegister - write failed");
            }
        }
    }

    

    // Gripper communicates over Modbus. This class maintains the TCP connection.
    class ModBus
    {
        private Stream s;
        private TcpClient client;
        private string address;
        public bool found;

        public ModBus(string address)
        {
            found = false;
            Console.WriteLine("Looking for gripper..." + address);
            if (this.connect(address))
            {
                found = true;
                Console.WriteLine("Gripper found!");
            }
            else
                Console.WriteLine("Gripper not found");
        }

        public bool connect(string address)
        {
            this.address = address;
            if (client != null && client.Connected) // Test if already connected
            {
                client.Close();
                client.Connect(address, 502);
                return true;
            }
            else
            {
                try
                {
                    client = new TcpClient(address, 502);
                    s = client.GetStream();
                    return true;
                }
                catch (SocketException e)
                {
                    //throw new Exception("Modbus connection failure");
                }
                return false;
            }
        }
        public void disconnect()
        {
            client.Close();
        }

        public bool sendCommand(Byte[] data, ref Byte[] received)
        {
            Byte[] rx_buffer = new Byte[30];
            int response_length = 0;
            try
            {
                s.Write(data, 0, data.Length);
            }
            catch
            {
                throw;
            }
            response_length = s.Read(rx_buffer, 0, 30);
            if (response_length == 0)
                return false;
            received = new Byte[response_length];
            Array.Copy(rx_buffer, received, response_length);
            return true;

        }
    }

    class GripperData
    {
        public byte[] currents;
        public byte[] positions;
        public byte completed;
        public byte mode;
        public byte a_detect;
        public byte b_detect;
        public byte c_detect;
        public byte scissors_detect;
        public byte found;
    }
}
