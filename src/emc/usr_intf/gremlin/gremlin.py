#!/usr/bin/env python
#    Copyright (C) 2009-2012
#    Jeff Epler <jepler@unpythonic.net>,
#    Pavel Shramov <psha@kamba.psha.org.ru>,
#    Chris Morley <chrisinnanaimo@hotmail.com>
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
#    2014 Steffen Noack
#    add property 'mouse_btn_mode'
#    0 = default: left rotate, middle move,   right zoom
#    1 =          left zoom,   middle move,   right rotate
#    2 =          left move,   middle rotate, right zoom
#    3 =          left zoom,   middle rotate, right move
#    4 =          left move,   middle zoom,   right rotate
#    5 =          left rotate, middle zoom,   right move
#
#    2015 Moses McKnight introduced mode 6 
#    6 = left move, middle zoom, right zoom (no rotate - for 2D plasma machines or lathes)
#
#    2016 Norbert Schechner
#    corrected mode handling for lathes, as in most modes it was not possible to move, as 
#    it has only been allowed in p view.


import gi
gi.require_version("Gtk","3.0")
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GL import shaders
import numpy as np

#import gtk
#import gtk.gtkgl.widget
#import gtk.gdkgl
#import gtk.gdk

import glnav
#import gobject
#import pango

import rs274.glcanon
import rs274.interpret
import linuxcnc
import gcode

import time
import re
import tempfile
import shutil
import os
import sys

import glm

import _thread

import pdb

# this helper function calculates a pixel transformation
# to the fitting object transformation
# vec3 oldpos: the position of the object to be moved
# uint dx, dy: the pixel delta
# mat4 model, proj: model, view, and projection matrices
# vec4 view: the current viewport (set with glViewport)
def px_delta_to_obj(oldpos, dx, dy, model, proj, view):
    # first get the px position of the current object position
    oldpos_px = glm.project(oldpos, model, proj, view)
    # calculate the new position of the object in window/pixel coordinates
    newpos_px = glm.vec3(oldpos_px.x + dx, oldpos_px.y + dy, 0)
    # transform the new position into object space
    newpos = glm.unProject(newpos_px, model, proj, view)
    # calculate the delta of the two positions
    return newpos - oldpos

"""
very simple vertex buffer object abstraction, no error checking performed currently (TODO)

the data_layout dictionary describes the layout of the data like this:

 'shader_name': {
    datatype: GL_TYPE_REPR,
    size: size of the data (bytes) / sizeof(typerepr)
    stride: stride of the data (bytes) between elements
    offset: offset of the data (bytes)
}
"""
class VBO():
    def __init__(self):
        self.vboid = -1
        self.data_layout = {
            'position': {
                'datatype': GL_FLOAT,
                'size': 3,
                'stride': 3*4*2,
                'offset': 0,
            },
            "color": {
                'datatype': GL_FLOAT,
                'size': 3,
                'stride': 3*4*2,
                'offset': 3*4,
            }
        }

    def gen(self):
        # Generate buffers to hold our vertices
        self.vboid = glGenBuffers(1)
        self.bind()

    def bind(self):
        glBindBuffer(GL_ARRAY_BUFFER, self.vboid)

    def unbind(self):
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    def bind_vao(self, shader):
        for shadername, layout in self.data_layout.items():
            location = glGetAttribLocation(shader, shadername)
            glEnableVertexAttribArray(location)
            # normalized currently unused & ignored
            glVertexAttribPointer(location, layout['size'], layout['datatype'], False, layout['stride'], ctypes.c_void_p(layout['offset']))

    def fill(self, data, size):
        self.bind()
        glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW)

class VAO():
    def __init__(self):
        self.vaoid=-1

    # this binds vbos with their respective bindings
    # to the used shader program
    def gen(self, shader, vbos):
        self.vaoid = glGenVertexArrays(1)
        self.bind()
        for vbo in vbos:

            vbo.bind_vao(shader)

    def bind(self):
        glBindVertexArray(self.vaoid)

    def unbind(self):
        glBindVertexArray(0)

class Camera():
    def __init__(self):
        # camera position
        self.position = glm.vec3(0,0,-10)
        self.lookpos = glm.vec3(0)

        # View settings
        self.zoomlevel = 10
        self.perspective = 0
        self.fovy = 30.0
        # Position of clipping planes.
        self.near = -1000.0
        self.far = 1000.0

        # projection matrix -> handles ortho / perspective
        self.proj = glm.mat4()
        # view matrix -> handles where the camera looks to/at
        self.view = glm.mat4()

        #self.lookat(self.lookpos)

    # returns the projection-view matrix for convenience
    def get(self):
        return self.proj * self.view

    def setpos(self, pos):
        self.position = pos
        self.update_view()

    def translate(self, delta):
        self.position += delta
        self.update_view()

    def zoom(self, level):
        self.zoomlevel *= level
        self.update_view()

    def update_view(self):
        self.view = glm.translate(glm.mat4(), self.position)
        self.view = glm.scale(self.view, glm.vec3(self.zoomlevel))

    """
    def lookat(self, center):
        self.lookpos = center
        # todo: check if the up vector is always (0,1,0)
        self.view = glm.lookAt(self.position, self.lookpos, glm.vec3(0.0,1.0,0))
    """

    # this method is called on resize and may not contain
    # any gl calls, because the context may not be bound
    # (generally, this class should only calc the matrix and
    # not change anything else)
    def update(self, w, h):
        self.w = w
        self.h = h

        if self.perspective:
            self.proj = glm.perspective(self.fovy,
                                       float(w)/float(h), # aspect ratio
                                       self.near, # clipping planes
                                       self.far)
        else:
            # left, right, bottom, top
            self.proj = glm.ortho(-w/2,w/2,-h/2,h/2,self.near,self.far)

# responsible for rendering everything scaled in the scene,
# like bounding box, axis, objects, etc.
class ObjectRenderer():
    def __init__(self):
        self.VERTEX_SOURCE = '''
        #version 330

        uniform mat4 proj; // view / projection matrix
        uniform mat4 model; // model matrix
        in vec4 position;
        in vec3 color;
        out vec3 v_color;

        void main() {
          gl_Position = proj * model * position;
          v_color = color;
        }'''

        self.FRAGMENT_SOURCE ='''
        #version 330

        in vec3 v_color;
        out vec3 out_color;

        void main() {
          out_color = v_color;
        }'''

        # used for emulating deprecated glLineStipple
        self.LINE_VERTEX_SOURCE = '''
        #version 330

        uniform mat4 proj; // view / projection matrix
        uniform mat4 model; // model matrix

        in vec4 position;
        in vec3 color;
        out vec3 v_color;
        flat out vec4 v_start; // does not get interpolated, because of flat
        out vec4 v_pos;

        void main() {
          v_pos = proj * model * position;
          v_start = v_pos;
          v_color = color;
        }'''

        self.LINE_FRAGMENT_SOURCE ='''
        #version 330

        flat in vec4 v_start;
        in vec4 v_pos;
        in vec3 v_color;
        out vec3 out_color;

        uniform vec2 u_resolution;
        uniform uint u_pattern;
        uniform float u_factor;

        void main() {
          out_color = v_color;

          vec2 dir = (v_pos.xy-v_start.xy) * u_resolution/2.0;
          float dist = length(dir);

          uint bit = uint(round(dist / u_factor)) & 15U;
          if ((u_pattern & (1U<<bit)) == 0U)
            discard;
        }'''


    def init(self):
        # generate shaders, get uniform locations
        VERTEX_SHADER_PROG = shaders.compileShader(self.VERTEX_SOURCE, GL_VERTEX_SHADER)
        FRAGMENT_SHADER_PROG = shaders.compileShader(self.FRAGMENT_SOURCE, GL_FRAGMENT_SHADER)
        self.shader_prog = shaders.compileProgram(VERTEX_SHADER_PROG, FRAGMENT_SHADER_PROG)
        self.projmat = glGetUniformLocation(self.shader_prog, "proj");
        self.modelmat = glGetUniformLocation(self.shader_prog, "model");

        # initial position / rotation / scale
        self.pos = glm.vec3(0,0,0)
        self.rotation = glm.mat4()
        self.scale = glm.vec3(100,100,100)
        # generate model matrix
        self._update_matrix()

        self.vbo = VBO()
        self.vao = VAO()

        self.vbo.gen()

        axes = np.array([
            # x axis
            1.0,0.0,0.0,
            0.2,1.0,0.2,
            0.0,0.0,0.0,
            0.2,1.0,0.2,
            # y axis
            0.0,1.0,0.0,
            1.0,0.2,0.2,
            0.0,0.0,0.0,
            1.0,0.2,0.2,
            # z axis
            0.0,0.0,1.0,
            0.2,0.2,1.0,
            0.0,0.0,0.0,
            0.2,0.2,1.0,
        ], dtype=np.float32)

        self.vbo.fill(axes, len(axes)*4)
        self.vao.gen(self.shader_prog, [self.vbo])

    def _update_matrix(self):
        newmat = glm.translate(glm.mat4(), self.pos)
        newmat *= self.rotation
        self.model = glm.scale(newmat, self.scale)

    # sets position / center of the scene to this location
    def move(self, pos):
        self.pos = pos
        self._update_matrix()

    def translate(self, delta):
        self.pos += delta
        self._update_matrix()

    def rotate(self, angle, axis):
        self.rotation = glm.rotate(self.rotation, angle, axis)
        self._update_matrix()

    # updates scale of the scene
    def scale(self, scale):
        self.scale = scale
        self._update_matrix()

    def render(self,projmat):
        self.vao.bind()

        glUseProgram(self.shader_prog)

        glUniformMatrix4fv(self.projmat, 1, GL_FALSE, glm.value_ptr(projmat));
        glUniformMatrix4fv(self.modelmat, 1, GL_FALSE, glm.value_ptr(self.model));

        '''
        print("matrices:")
        print(self.camera.get())
        print(self.model)
        '''

        glDrawArrays(GL_LINES, 0,2)
        glDrawArrays(GL_LINES, 2,2)
        glDrawArrays(GL_LINES, 4,2)

        self.vao.unbind()

        glUseProgram(0)


class Gremlin(Gtk.GLArea):

    def init_glcanondraw(self):
        pass
    def activate(self):
        pass
    def deactivate(self):
        pass
    def set_current_view(self):
        pass

    def __init__(self, inifile):
        self.initialised = True
        self.lathe_option = None

        Gtk.GLArea.__init__(self)

        self.set_auto_render(True)

        # save mouseposition, because
        # it's the only way to get a position delta
        # (thankyou gtk)
        self.mouse_x = 0
        self.mouse_y = 0

        # camera is for managing the projection and view matrices
        self.camera = Camera()
        self.camera.setpos(glm.vec3(0,0,-5))

        self.connect("realize", self.on_realize)
        self.connect("render", self.on_render)
        self.connect("resize", self.on_resize)
        # mouse events
        self.connect('motion-notify-event', self.on_motion)
        self.connect("button-press-event", self.on_button_pressed)
        self.connect("button-release-event", self.on_button_released)
        self.connect("scroll-event", self.on_scroll)

        self.add_events(Gdk.EventMask.POINTER_MOTION_MASK)
        self.add_events(Gdk.EventMask.POINTER_MOTION_HINT_MASK)
        self.add_events(Gdk.EventMask.SCROLL_MASK)
        self.add_events(Gdk.EventMask.BUTTON_MOTION_MASK)
        self.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.add_events(Gdk.EventMask.BUTTON_RELEASE_MASK)

        self.object_renderer = ObjectRenderer()

    def on_motion(self, widget, event):
        button1 = event.state & Gdk.ModifierType.BUTTON1_MASK
        button2 = event.state & Gdk.ModifierType.BUTTON2_MASK
        button3 = event.state & Gdk.ModifierType.BUTTON3_MASK
        shift = event.state & Gdk.ModifierType.SHIFT_MASK

#        print(f"{button1} {button2} {button3} {shift}")
        d_x = self.mouse_x - event.x
        d_y = self.mouse_y - event.y

        if button1:
            self.object_renderer.rotate(d_x*0.5, glm.vec3(1,0,0))
            self.object_renderer.rotate(d_y*0.5, glm.vec3(0,1,0))

        if button3:
            self.camera.translate(glm.vec3(-d_x, d_y, 0))

        self.mouse_x = event.x
        self.mouse_y = event.y

    def on_button_pressed(self, widget, event):
        pass

    def on_button_released(self, widget, event):
        pass

    def on_scroll(self, widget, event):
        if event.direction == Gdk.ScrollDirection.DOWN:
            self.camera.zoom(1.1)
        elif event.direction == Gdk.ScrollDirection.UP:
            self.camera.zoom(1/1.1)

    # initialization of all gl objects, as per gtk documentation
    # the opengl context is initialized at this point
    def on_realize(self, area):
        ctx = self.get_context()
        ctx.make_current()

        self.on_resize(area,self.get_allocated_width(),self.get_allocated_height())

        glClearColor(0, 0, 0, 1)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.object_renderer.init()

    # the projection matrices need to be updated on resize
    def on_resize(self, area, width, height):
        self.w = width
        self.h = height

        self.camera.update(self.w,self.h)

    def on_render(self, area, context):
        # gtk doc says area not context
        area.make_current()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glViewport(0,0,self.w,self.h)

        self.object_renderer.render(self.camera.get())

        # render every frame, #yolo
        self.queue_render()

"""
class DummyProgress:
    def nextphase(self, unused): pass
    def progress(self): pass

class StatCanon(rs274.glcanon.GLCanon, rs274.interpret.StatMixin):
    def __init__(self, colors, geometry, lathe_view_option, stat, random):
        rs274.glcanon.GLCanon.__init__(self, colors, geometry)
        rs274.interpret.StatMixin.__init__(self, stat, random)
        self.progress = DummyProgress()
        self.lathe_view_option = lathe_view_option

    def is_lathe(self): return self.lathe_view_option

    def change_tool(self, pocket):
        rs274.glcanon.GLCanon.change_tool(self,pocket)
        rs274.interpret.StatMixin.change_tool(self,pocket)




class Gremlin(Gtk.GLArea):#,rs274.glcanon.GlCanonDraw,glnav.GlNavBase):
    rotation_vectors = [(1.,0.,0.), (0.,0.,1.)]

    def __init__(self, inifile):
        Gtk.GLArea.__init__(self)
        self.set_has_depth_buffer(True)
        #self.set_has_alpha(True)
        #'set_has_stencil_buffer',
        glutInit()
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH )


#class Gremlin(gtk.gtkgl.widget.DrawingArea, glnav.GlNavBase,
#              rs274.glcanon.GlCanonDraw):
#    rotation_vectors = [(1.,0.,0.), (0., 0., 1.)]

#    def __init__(self, inifile):
#
#        display_mode = ( gtk.gdkgl.MODE_RGB | gtk.gdkgl.MODE_DEPTH |
#                         gtk.gdkgl.MODE_DOUBLE )
#        glconfig = gtk.gdkgl.Config(mode=display_mode)

#        gtk.gtkgl.widget.DrawingArea.__init__(self, glconfig)
        glnav.GlNavBase.__init__(self)
        def C(s):
            a = self.colors[s + "_alpha"]
            s = self.colors[s]
            return [int(x * 255) for x in s + (a,)]
        self.inifile = inifile
        self.logger = linuxcnc.positionlogger(linuxcnc.stat(),
            C('backplotjog'),
            C('backplottraverse'),
            C('backplotfeed'),
            C('backplotarc'),
            C('backplottoolchange'),
            C('backplotprobing'),
            self.get_geometry()
        )
        _thread.start_new_thread(self.logger.start, (.01,))

        rs274.glcanon.GlCanonDraw.__init__(self, linuxcnc.stat(), self.logger)

        self.current_view = 'z'

        self.select_primed = None

        self.connect_after('realize', self.realize)
        self.connect('configure_event', self.reshape)
        self.connect('map_event', self.map)
        #self.connect('draw', self.expose) # expose_event was deprecated
        self.connect('motion-notify-event', self.motion)
        self.connect('button-press-event', self.pressed)
        self.connect('button-release-event', self.select_fire)
        self.connect('scroll-event', self.scroll)

        self.add_events(Gdk.EventMask.POINTER_MOTION_MASK)
        self.add_events(Gdk.EventMask.POINTER_MOTION_HINT_MASK)
        #self.add_events(gdk.BUTTON_MOTION_MASK)
        #self.add_events(gdk.EventMask.BUTTON_PRESS_MASK)
        #self.add_events(gdk.BUTTON_RELEASE_MASK)
        self.add_events(Gdk.EventMask.BUTTON_MOTION_MASK)
        self.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.add_events(Gdk.EventMask.BUTTON_RELEASE_MASK)
 

        self.fingerprint = ()

        self.lat = 0
        self.minlat = -90
        self.maxlat = 90

        self.highlight_line = None
        self.program_alpha = False
        self.use_joints_mode = False
        self.use_commanded = True
        self.show_limits = True
        self.show_extents_option = True
        self.show_live_plot = True
        self.show_velocity = True
        self.metric_units = True
        self.show_program = True
        self.show_rapids = True
        self.use_relative = True
        self.show_tool = True
        self.show_dtg = True
        self.grid_size = 0.0
        temp = inifile.find("DISPLAY", "LATHE")
        self.lathe_option = bool(temp == "1" or temp == "True" or temp == "true" )
        self.foam_option = bool(inifile.find("DISPLAY", "FOAM"))
        self.show_offsets = False
        self.use_default_controls = True
        self.mouse_btn_mode = 0

        self.a_axis_wrapped = inifile.find("AXIS_A", "WRAPPED_ROTARY")
        self.b_axis_wrapped = inifile.find("AXIS_B", "WRAPPED_ROTARY")
        self.c_axis_wrapped = inifile.find("AXIS_C", "WRAPPED_ROTARY")

        live_axis_count = 0
        for i,j in enumerate("XYZABCUVW"):
            if self.stat.axis_mask & (1<<i) == 0: continue
            live_axis_count += 1
        self.num_joints = int(inifile.find("KINS", "JOINTS") or live_axis_count)
        glDrawBuffer(GL_BACK)
        glDisable(GL_CULL_FACE)
        glLineStipple(2, 0x5555)
        glDisable(GL_LIGHTING)
        glClearColor(0,0,0,0)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)



    def activate(self):
        self.make_current()
        #glcontext = gtk.gtkgl.widget_get_gl_context(self)
        #gldrawable = gtk.gtkgl.widget_get_gl_drawable(self)

        #return gldrawable and glcontext and gldrawable.gl_begin(glcontext)
        return True

    def swapbuffers(self):
        #gldrawable = gtk.gtkgl.widget_get_gl_drawable(self)
        #gldrawable.swap_buffers()
        return

    def deactivate(self):
        return
        #TODO
        #gldrawable = Gtk.gtkgl.widget_get_gl_drawable(self)
        #gldrawable.gl_end()

    def winfo_width(self):
        return  self.get_allocated_width()

    def winfo_height(self):
        return self.get_allocated_height()

    def reshape(self, widget, event):
        self.width = event.width
        self.height = event.height

    def expose(self, widget=None, event=None):
        if not self.initialised: return
        if self.perspective: self.redraw_perspective()
        else: self.redraw_ortho()

        return True

    def _redraw(self):
        print("yolo")
        self.expose()

    def clear_live_plotter(self):
        self.logger.clear()

    def map(self, *args):
        GObject.timeout_add(50, self.poll)

    def poll(self):
        s = self.stat
        try:
            s.poll()
        except:
            return
        fingerprint = (self.logger.npts, self.soft_limits(),
            s.actual_position, s.joint_actual_position,
            s.homed, s.g5x_offset, s.g92_offset, s.limit, s.tool_in_spindle,
            s.motion_mode, s.current_vel)

        if fingerprint != self.fingerprint:
            self.fingerprint = fingerprint
            self.queue_draw()

        # return self.visible
        return True

    @rs274.glcanon.with_context
    def realize(self, widget):
        self.make_current()
        self.set_current_view()
        s = self.stat
        try:
            s.poll()
        except Exception as e:
            print(e)
            return
        self._current_file = None

        self.font_base, width, linespace = \
                glnav.use_pango_font('courier bold 16', 0, 128)
        self.font_linespace = linespace
        self.font_charwidth = width
        rs274.glcanon.GlCanonDraw.realize(self)

        if s.file: self.load()

    def set_current_view(self):
        if self.current_view not in ['p', 'x', 'y', 'y2', 'z', 'z2']:
            return
        return getattr(self, 'set_view_%s' % self.current_view)()

    def load(self,filename = None):
        s = self.stat
        s.poll()
        if not filename and s.file:
            filename = s.file
        elif not filename and not s.file:
            return

        td = tempfile.mkdtemp()
        self._current_file = filename
        try:
            random = int(self.inifile.find("EMCIO", "RANDOM_TOOLCHANGER") or 0)
            canon = StatCanon(self.colors, self.get_geometry(),self.lathe_option, s, random)
            parameter = self.inifile.find("RS274NGC", "PARAMETER_FILE")
            temp_parameter = os.path.join(td, os.path.basename(parameter or "linuxcnc.var"))
            if parameter:
                shutil.copy(parameter, temp_parameter)
            canon.parameter_file = temp_parameter

            unitcode = "G%d" % (20 + (s.linear_units == 1))
            initcode = self.inifile.find("RS274NGC", "RS274NGC_STARTUP_CODE") or ""
            result, seq = self.load_preview(filename, canon, unitcode, initcode)
            if result > gcode.MIN_ERROR:
                self.report_gcode_error(result, seq, filename)

        finally:
            shutil.rmtree(td)

        self.set_current_view()

    def get_program_alpha(self): return self.program_alpha
    def get_num_joints(self): return self.num_joints
    def get_geometry(self):
        temp = self.inifile.find("DISPLAY", "GEOMETRY")
        if temp:
            geometry = re.split(" *(-?[XYZABCUVW])", temp.upper())
            self.geometry = "".join(reversed(geometry))
        else:
            self.geometry = 'XYZ'
        return self.geometry

    def get_joints_mode(self): return self.use_joints_mode
    def get_show_commanded(self): return self.use_commanded
    def get_show_extents(self): return self.show_extents_option
    def get_show_limits(self): return self.show_limits
    def get_show_live_plot(self): return self.show_live_plot
    def get_show_machine_speed(self): return self.show_velocity
    def get_show_metric(self): return self.metric_units
    def get_show_program(self): return self.show_program
    def get_show_rapids(self): return self.show_rapids
    def get_show_relative(self): return self.use_relative
    def get_show_tool(self): return self.show_tool
    def get_show_distance_to_go(self): return self.show_dtg
    def get_grid_size(self): return self.grid_size

    def get_view(self):
        view_dict = {'x':0, 'y':1, 'y2':1, 'z':2, 'z2':2, 'p':3}
        return view_dict.get(self.current_view, 3)

    def is_lathe(self): return self.lathe_option
    def is_foam(self): return self.foam_option
    def get_current_tool(self):
        for i in self.stat.tool_table:
            if i[0] == self.stat.tool_in_spindle:
                return i
    def get_highlight_line(self): return self.highlight_line

    def get_a_axis_wrapped(self): return self.a_axis_wrapped
    def get_b_axis_wrapped(self): return self.b_axis_wrapped
    def get_c_axis_wrapped(self): return self.c_axis_wrapped

    def get_font_info(self):
        return self.font_charwidth, self.font_linespace, self.font_base

    def get_show_offsets(self): return self.show_offsets

    def select_prime(self, x, y):
        self.select_primed = x, y

    @rs274.glcanon.with_context
    def select_fire(self, widget, event):
        if not self.select_primed: return
        x, y = self.select_primed
        self.select_primed = None
        self.select(x, y)

    def select_cancel(self, widget=None, event=None):
        self.select_primed = None

    def pressed(self, widget, event):
        if not self.use_default_controls:return
        button1 = event.button == 1
        button2 = event.button == 2
        button3 = event.button == 3
        if button1:
            self.select_prime(event.x, event.y) # select G-Code element
        
        if button3 and (event.type == Gdk.EventType._2BUTTON_PRESS):
            self.clear_live_plotter()
        elif button1 or button2 or button3:
            self.startZoom(event.y)
            self.recordMouse(event.x, event.y)

    def motion(self, widget, event):
        if not self.use_default_controls:return
        button1 = event.state & Gdk.ModifierType.BUTTON1_MASK
        button2 = event.state & Gdk.ModifierType.BUTTON2_MASK
        button3 = event.state & Gdk.ModifierType.BUTTON3_MASK
        shift = event.state & Gdk.ModifierType.SHIFT_MASK
        # for lathe or plasmas rotation is not used, so we check for it
        # recomended to use mode 6 for that type of machines
        cancel = bool(self.lathe_option)
        
        # 0 = default: left rotate, middle move, right zoom
        if self.mouse_btn_mode == 0:
            if button1:
                if shift:
                    self.translateOrRotate(event.x, event.y)
                elif not cancel:
                    self.set_prime(event.x, event.y)
                    self.rotateOrTranslate(event.x, event.y)
            elif button2:
                self.translateOrRotate(event.x, event.y)
            elif button3:
                self.continueZoom(event.y)
        # 1 = left zoom, middle move, right rotate
        elif self.mouse_btn_mode == 1:
            if button1:
                if shift:
                    self.translateOrRotate(event.x, event.y)
                else:
                    self.continueZoom(event.y)
            elif button2:
                self.translateOrRotate(event.x, event.y)
            elif button3 and not cancel:
                self.set_prime(event.x, event.y)
                self.rotateOrTranslate(event.x, event.y)
        # 2 = left move, middle rotate, right zoom
        elif self.mouse_btn_mode == 2:
            if button1:    
                if shift:
                    if not cancel:
                        self.set_prime(event.x, event.y)
                        self.rotateOrTranslate(event.x, event.y)
                else:
                    self.translateOrRotate(event.x, event.y)
            elif button2 and not cancel:
                self.set_prime(event.x, event.y)
                self.rotateOrTranslate(event.x, event.y)
            elif button3:
                self.continueZoom(event.y)
        # 3 = left zoom, middle rotate, right move
        elif self.mouse_btn_mode == 3:
            if button1:    
                if shift:
                    if not cancel:
                        self.set_prime(event.x, event.y)
                        self.rotateOrTranslate(event.x, event.y)
                else:
                    self.continueZoom(event.y)
            elif button2 and not cancel:
                self.set_prime(event.x, event.y)
                self.rotateOrTranslate(event.x, event.y)
            elif button3:
                self.translateOrRotate(event.x, event.y)
        # 4 = left move,   middle zoom,   right rotate
        elif self.mouse_btn_mode == 4:
            if button1:    
                if shift:
                    if not cancel:
                        self.set_prime(event.x, event.y)
                        self.rotateOrTranslate(event.x, event.y)
                else:
                    self.translateOrRotate(event.x, event.y)
            elif button2:
                self.continueZoom(event.y)
            elif button3 and not cancel:
                self.set_prime(event.x, event.y)
                self.rotateOrTranslate(event.x, event.y)
        # 5 = left rotate, middle zoom, right move
        elif self.mouse_btn_mode == 5:
            if button1:    
                if shift:
                    self.continueZoom(event.y)
                elif not cancel:
                    self.set_prime(event.x, event.y)
                    self.rotateOrTranslate(event.x, event.y)
            elif button2:
                self.continueZoom(event.y)
            elif button3:
                self.translateOrRotate(event.x, event.y)
        # 6 = left move, middle zoom, right zoom (no rotate - for 2D plasma machines or lathes)
        elif self.mouse_btn_mode == 6:
            if button1:    
                if shift:
                    self.continueZoom(event.y)
                else:
                    self.translateOrRotate(event.x, event.y)
            elif button2:
                self.continueZoom(event.y)
            elif button3:
                self.continueZoom(event.y)

    def scroll(self, widget, event):
        if not self.use_default_controls:return
        if event.direction == Gdk.EventType.SCROLL_UP: self.zoomin()
        elif event.direction == Gdk.EventType.SCROLL_DOWN: self.zoomout()

    def report_gcode_error(self, result, seq, filename):

        error_str = gcode.strerror(result)
        sys.stderr.write("G-Code error in " + os.path.basename(filename) + "\n" + "Near line "
                         + str(seq) + " of\n" + filename + "\n" + error_str + "\n")

    # These are for external controlling of the view

    def zoom_in(self):
        self.zoomin()

    def zoom_out(self):
        self.zoomout()

    def start_continuous_zoom(self, y):
        self.startZoom(y)

    def continuous_zoom(self, y):
        self.continueZoom(y)

    def set_mouse_start(self, x, y):
        self.recordMouse(x, y)

    def set_prime(self, x, y):
        if self.select_primed:
            primedx, primedy = self.select_primed
            distance = max(abs(x - primedx), abs(y - primedy))
            if distance > 8: self.select_cancel()

    def pan(self,x,y):
        self.translateOrRotate(x, y)

    def rotate_view(self,x,y):
        self.rotateOrTranslate(x, y)
"""
