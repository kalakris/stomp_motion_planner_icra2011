#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

PKG = 'dmp_motion_gui'
import roslib; roslib.load_manifest(PKG)

import rospy
import rosparam
import rosservice

import os
import optparse
import sys
import time

import rosbag
from sensor_msgs.msg import JointState

import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)
import wx

class DmpMotionGuiApp(wx.App):
    def __init__(self, options):
        self.options = options
        self.bag = None
        self.sub = None
        wx.App.__init__(self)
    
    def OnInit(self):
        frame = DmpMotionFrame(None, self.options)
        frame.Show()
        self.SetTopWindow(frame)
        return True

class DmpMotionFrame(wx.Frame):
    def __init__(self, parent, options, id=wx.ID_ANY, title='DMP Motion', pos=wx.DefaultPosition, size=(700, 840), style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)

        self.options = options

        self.font = wx.Font(9, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

        self.dmps      = []
        self.queue     = []
        self.robots    = ['sim', 'pre2', 'prf2', 'prg2']
        self.robot     = 'sim'
        self.colors    = ['red', 'blue', 'purple', 'cyan', 'brown']

        self.controller_name = 'dmp_joint_position_controller'
        self.what = 'learn_joint_space_dmp_from_bag_file'

        self.filename   = None
        self.recording  = False
        self.record_pid = None

        self._create_menus()
        self._create_gui()
        self._populate_gui()

    def _create_menus(self):
        self.menu_bar = wx.MenuBar()

        # File

        self.file_menu = wx.Menu()
        self.file_exit_item = wx.MenuItem(self.file_menu, -1, 'E&xit', 'Exit the program')
        self.Bind(wx.EVT_MENU, self.on_exit, self.file_exit_item)
        self.file_menu.AppendItem(self.file_exit_item)
        self.menu_bar.Append(self.file_menu, '&File')

        # Robot

        self.robot_menu = wx.Menu()

        class RobotMenuItem(wx.MenuItem):
            def __init__(self, parent, robot):
                wx.MenuItem.__init__(self, parent, -1, robot, 'Use robot ' + robot, kind=wx.ITEM_CHECK)
                self.robot = robot
        
        for robot in self.robots:
            robot_item = RobotMenuItem(self.robot_menu, robot)
            self.Bind(wx.EVT_MENU, self.on_robot, robot_item)
            self.robot_menu.AppendItem(robot_item)
        self.menu_bar.Append(self.robot_menu, '&Robot')

        self.SetMenuBar(self.menu_bar)

    def _create_gui(self):
        self.output_text            = wx.TextCtrl  (self, -1, '',                      pos=(  0, 360), size=(200, 200), style=wx.TE_MULTILINE)
        self.record_bag_text        = wx.TextCtrl  (self, -1, 'record.bag',            pos=(  5,   5), size=(180,  29))
        self.record_button          = wx.Button    (self, -1, 'Start Record',          pos=(200,   5))
        self.bag_combo              = wx.ComboBox  (self, -1, 'Bag Files',             pos=(  5,  45), size=(180,  29), style=wx.CB_READONLY)
        self.learn_button           = wx.Button    (self, -1, 'Learn DMP',             pos=(200,  45))
        self.reload_library_button  = wx.Button    (self, -1, 'Reload Library',        pos=(300,  45))
        self.control_combo          = wx.ComboBox  (self, -1, 'Library',               pos=(  5,  80), size=(180,  29))
        self.color_label            = wx.StaticText(self, -1, 'Color:',                pos=(200,  86), size=(100,  29))
        self.color_combo            = wx.ComboBox  (self, -1, '',                      pos=(240,  80), size=( 90,  29), style=wx.CB_READONLY)
        self.duration_label         = wx.StaticText(self, -1, 'Duration:',             pos=(340,  86))
        self.duration_text          = wx.TextCtrl  (self, -1, '3.5',                  pos=(405,  80), size=( 60,  29), style=wx.ALIGN_RIGHT)
        self.setup_button           = wx.Button    (self, -1, 'Setup',                 pos=(480,  80), size=( 55,  29))
        self.reproduction_button    = wx.Button    (self, -1, 'Setup on Initial Goal', pos=(540,  80), size=(145,  29))
        self.print_queue_button     = wx.Button    (self, -1, 'Print Queue',           pos=(200, 120))
        self.clear_queue_button     = wx.Button    (self, -1, 'Clear Queue',           pos=(295, 120))
        self.queue_combo            = wx.ComboBox  (self, -1, 'Setup Queue',           pos=(  5, 120), size=(180,  29))
        self.add_run_queue_button   = wx.Button    (self, -1, 'Add',                   pos=(  5, 315))
        self.clear_run_queue_button = wx.Button    (self, -1, 'Clear',                 pos=( 95, 315))
        self.queue_list             = wx.ListBox   (self, -1,                          pos=(  5, 160), size=(180, 150))
        self.goto_start_button      = wx.Button    (self, -1, 'Goto Start',            pos=(400, 120))
        self.switch_to_dmp_controller_button = wx.Button(self, -1, 'Switch to DMP controller', pos=(495, 120))

        self.run_button             = wx.Button    (self, -1, 'Run',                   pos=(530, 315))

        self.output_text.SetFont(self.font)
        
        self.run_button.SetBackgroundColour(wx.RED)
        self.run_button.SetForegroundColour(wx.WHITE)

        self.Bind(wx.EVT_SIZE, self.on_size)

        self.output_text.Bind(wx.EVT_TEXT, self.on_text)

        self.Bind(wx.EVT_BUTTON, lambda e: self.toggle_recording(),   self.record_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.learn(),              self.learn_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.load_dmp_library(),   self.reload_library_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.setup(),              self.setup_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.setup_reproduction(), self.reproduction_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.call_queue('print'),  self.print_queue_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.clear_queue(),        self.clear_queue_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.add_run_queue(),      self.add_run_queue_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.goto_start(),         self.goto_start_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.switch_to_dmp_controller(), self.switch_to_dmp_controller_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.clear_run_queue(),    self.clear_run_queue_button)
        self.Bind(wx.EVT_BUTTON, lambda e: self.run(),                self.run_button)
        
    def on_robot(self, event):
        print event.ClientObject
    
    def on_exit(self, event):
        self.Close()
    
    def _populate_gui(self):
        self.color_combo.SetItems(self.colors)

        if not self.options.offline:
            # TODO: add exception handling when param is not set
            self.bag_dir = rosparam.get_param('/dmp_motion_learner/dmp/data_directory_name')
            self.populate_bag_combo()
            self.load_dmp_library()
            self.load_queue()

    def populate_bag_combo(self):
        bags = os.listdir(self.bag_dir)
        self.bag_combo.SetItems(sorted(bags))
    
    ## Recording
    
    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()
    
    def start_recording(self):
        if self.recording:
            return
        
        self.record_button.SetLabel('Stop Record')

        self.filename = self.record_bag_text.GetValue()
        # if self.filename.endswith('.bag'):
        #    self.filename.replace('.bag', '')
        
        self.recording = True

        self.bag = rosbag.Bag(self.bag_dir + '/' + self.filename, 'w')
        self.sub = rospy.Subscriber('/joint_states', JointState, self.record_callback)

        # cmd = 'rosrecord /joint_states -F ' + self.bag_dir + '/' + self.filename
        # print cmd
        # self.record_pid = subprocess.Popen(cmd, shell=True).pid

    def record_callback(self, msg):
        self.bag.write('/joint_states', msg)

    def stop_recording(self):
        if not self.recording:
            return

        self.sub.unregister()
        self.record_button.SetLabel('Start Record')

        # Kill it
#        import signal
#        os.kill(self.record_pid, signal.SIGINT)

        # Wait before copying the file
        time.sleep(0.1)

        self.bag.close()

        # Copy the file onto the robot
        if self.robot != 'sim':
            os.popen('scp ' + self.bag_dir + '/' + self.filename + '.bag' + ' ' + self.robot + ':' + self.bag_dir)

        # Repopulate the bag file combo
        self.populate_bag_combo()

        self.recording = False

    def add_run_queue(self):
        selected = str(self.queue_combo.GetValue())
        
        self.queue_list.Append(selected)
        
    def clear_run_queue(self):        
        self.queue_list.Clear()
        
    def get_dmp_names(self, dmps):
        return ['%s: %s' % (dmp['id'], dmp['source file name']) for dmp in dmps if 'id' in dmp and 'source file name' in dmp]
        
    def setup(self):
        id, name = str(self.control_combo.GetValue()).split(': ')
        id       = int(id)
        color    = str(self.color_combo.GetValue())
        duration = float(self.duration_text.GetValue())
        
        self.call_service('/' + self.controller_name + '/setup_dmp_on_color_blob', [id], [color])
        self.call_service('/' + self.controller_name + '/setup_dmp_duration',      [id], [duration])        
        self.load_queue()
        
    def setup_reproduction(self):
        id, name = str(self.control_combo.GetValue()).split(': ')
        
        id       = int(id)
        duration = float(self.duration_text.GetValue())
        
        self.call_service('/' + self.controller_name + '/setup_dmp_for_reproduction', [id])
        self.call_service('/' + self.controller_name + '/setup_dmp_duration',         [id], [duration])
        self.load_queue()

    def call_service(self, service, *args):
        req, resp = rosservice.call_service(service, list(args))        

        self.append_output(str(resp))
        
        return resp
        
    def clear_queue(self):
        self.call_queue('clear')
        self.load_queue()

    def goto_start(self):
        resp = self.call_service('/dmp_task_learner/goto_start')
        return str(resp)

    def switch_to_dmp_controller(self):
        resp = self.call_service('/dmp_task_learner/run_trial')
        return str(resp)

    def load_queue(self):
        resp = self.call_queue('print')

        dmps = []
        dmp = {}
        for line in resp.split('\n'):
            fields = line.split('\t')
            for field in fields:
                values = field.split(': ')
                if len(values) == 2:
                    key, value = values
                    
                    dmp[key] = value
                    if key == 'type':
                        dmps.append(dmp)
                        dmp = {}

        self.queue = dmps
        
        self.queue_combo.SetItems(self.get_dmp_names(self.queue))

    def call_queue(self, cmd):
        resp = self.call_service('/' + self.controller_name +'/setup_dmp_queue', cmd)
        
        return str(resp)

    def run(self):
        items = self.queue_list.GetItems()
        ids = [int(item.split(':')[0]) for item in items]
        
        if len(ids) > 0:
            self.call_service('/' + self.controller_name + '/run_dmp', ids)

    def learn(self):
        bag_file = str(self.bag_combo.GetValue())

        self.learn_dmp(bag_file)

    def on_size(self, event):
        x, y = self.output_text.GetPosition()
        w, h = self.GetClientSize()
        
        self.output_text.SetSize((w, h - y))
        
        self.Refresh()
    
    def on_text(self, event):
        # Scroll to end
        self.output_text.ScrollPages(1)
    
    def append_output(self, output):
        self.output_text.AppendText(output + '\n')
            
    def learn_dmp(self, filename):
#        resp = self.call_service('/' + self.what, 
#            'torso_lift_link',
#            'r_gripper_tool_frame',
#            ['r_shoulder_pan', 'r_shoulder_lift', 'r_upper_arm_roll', 'r_elbow_flex', 'r_forearm_roll', 'r_wrist_flex', 'r_wrist_roll'],
#            filename)
        resp = self.call_service('/' + self.what, 
            ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
            filename)

        self.append_output(str(resp))
        
        # Refresh the library
        self.load_dmp_library()
        
    def load_dmp_library(self):
        resp = self.call_service('/print_dmp_library', 'quiet')
        
        # Parse the output from /print_dmp_library
        msg = str(resp)
        msg = msg.replace('info: DMP Library content:','')
        msg = msg.replace('\nid:', '\tid:')
        lines = msg.split('\t')
        dmps = []
        dmp = {}
        for i, line in enumerate(lines):
            for field in ['id', 'name', 'source file name', 'description', 'type']:
                if field in line:
                    dmp[field] = line[line.index(field) + len(field) + 2:].replace('\n', '')
                    if field == 'type':
                        dmps.append(dmp)
                        dmp = {}
                        
        self.dmps = dmps

        # Repopulate the dropdown
        self.control_combo.SetItems(self.get_dmp_names(self.dmps))

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('-o', '--offline', action='store_true', default=False, help='run offline - don\'t connect to core')   
    options, args = parser.parse_args(sys.argv[1:])

    if not options.offline:
        rospy.init_node('dmp_motion_gui', anonymous=True)

    app = DmpMotionGuiApp(options)
    app.MainLoop()
 
    rospy.signal_shutdown('GUI shutdown')    
