import arcade
import math

from safety_shield.utils import Quantizer, str2list, list2str

# python class to visualize the arena
class ArenaVisualizer(arcade.Window):
    def __init__(self, car_position_relative_to_arena_base, config_reader, get_dynamic_sets):

        # callback to get the dynamic objects
        self.get_dynamic_sets = get_dynamic_sets

        # load the configurations
        self.config_reader = config_reader
        self.load_configs()

        # initialize the arcade thing
        super().__init__(self.SCREEN_WIDTH, self.SCREEN_HEIGHT, "AV-Arena-Simulator: " + self.title)
        
        # crete the system sprite
        self.system = arcade.Sprite(self.model_image, self.model_image_scale)
        self.last_action = [0.0, 0.0]
        self.last_action_symbol = -1

        # set system's initial state in arrena
        self.x_0 = car_position_relative_to_arena_base
        x_0_arena = self.translate_sys_to_arena(self.x_0)
        self.system.center_x = x_0_arena[0]
        self.system.center_y = x_0_arena[1]
        self.system.angle = self.x_0[2]*180.0/math.pi

        # prepare for simulation
        self.sys_state = self.x_0
        self.sys_status = "stopped"
        self.initial_delay = 100
        self.time_elapsed = 0.0
        self.avg_delta = 0.0
        self.total_sim_time = 0.0
        arcade.set_background_color(arcade.color.WHITE)

    def load_configs(self):
        # get some values form the config reader
        self.x_lb = str2list(self.config_reader.get_value_string("states.lb"))
        self.x_ub = str2list(self.config_reader.get_value_string("states.ub"))
        self.x_eta = str2list(self.config_reader.get_value_string("states.eta"))

        self.SCREEN_WIDTH = 1000
        self.SCREEN_HEIGHT = 250
        self.title = "pFaces-SymControl Arena Visualizer"
        self.model_image = "safety_shield\\car.png"
        self.model_image_scale = 0.06

        # create a quantizer for the state space
        self.qnt_x = Quantizer(self.x_lb, self.x_eta, self.x_ub)

        # configs for the arena
        self.PADDING = 40
        self.ZERO_BASE_X = self.PADDING
        self.ZERO_BASE_Y = self.PADDING
        self.X_GRID = (self.SCREEN_WIDTH-2*self.PADDING)/self.qnt_x.get_widths()[0]
        self.Y_GRID = (self.SCREEN_HEIGHT-2*self.PADDING)/self.qnt_x.get_widths()[1]
        self.ARENA_WIDTH = (self.qnt_x.get_widths()[0])*self.X_GRID
        self.ARENA_HIGHT = (self.qnt_x.get_widths()[1])*self.Y_GRID
        self.X_SCALE_FACTOR = self.ARENA_WIDTH/(self.x_ub[0] - self.x_lb[0] + self.x_eta[0])
        self.Y_SCALE_FACTOR = self.ARENA_HIGHT/(self.x_ub[1] - self.x_lb[1] + self.x_eta[1])
        self.arena_mdl_lb = self.translate_sys_to_arena(self.x_lb)
        self.arena_mdl_ub = self.translate_sys_to_arena(self.x_ub)

    def translate_sys_to_arena(self, state):
        arena_x = self.ZERO_BASE_X + (state[0] - self.x_lb[0] + self.x_eta[0]/2)*self.X_SCALE_FACTOR
        arena_y = self.ZERO_BASE_Y + (state[1] - self.x_lb[1] + self.x_eta[1]/2)*self.Y_SCALE_FACTOR
        return [arena_x, arena_y]

    def draw_arena(self):
        
        # Draw the background
        arcade.draw_rectangle_filled(self.ZERO_BASE_X + self.ARENA_WIDTH/2, self.ZERO_BASE_Y + self.ARENA_HIGHT/2, self.ARENA_WIDTH, self.ARENA_HIGHT, arcade.color.LIGHT_GRAY)
        
        # draw grid
        num_x_lines = self.qnt_x.get_widths()[0] + 1
        for i in range(num_x_lines):
            arcade.draw_line(self.ZERO_BASE_X+i*self.X_GRID, self.ZERO_BASE_Y, self.ZERO_BASE_X+i*self.X_GRID, self.ZERO_BASE_Y+self.ARENA_HIGHT, arcade.color.BLACK)
        num_y_lines = self.qnt_x.get_widths()[1] + 1
        for i in range(num_y_lines):
            arcade.draw_line(self.ZERO_BASE_X, self.ZERO_BASE_Y+i*self.Y_GRID, self.ZERO_BASE_X+self.ARENA_WIDTH, self.ZERO_BASE_Y+i*self.Y_GRID, arcade.color.BLACK)        
        
        # draw lb/ub markers
        x_first = self.translate_sys_to_arena(self.x_lb)
        arcade.draw_rectangle_filled(x_first[0], x_first[1], 5, 5, arcade.color.BLUE)
        arcade.draw_text("x_first", x_first[0], x_first[1], arcade.color.BLACK, 14)
        x_last = self.translate_sys_to_arena(self.x_ub)
        arcade.draw_rectangle_filled(x_last[0], x_last[1], 5, 5, arcade.color.BLUE)
        arcade.draw_text("x_last", x_last[0], x_last[1], arcade.color.BLACK, 14)

        # collect dynamic sets
        dynamic_sets = self.get_dynamic_sets()

        # draw dynamic objects (obstacles)
        dynamic_objects = dynamic_sets[0]
        for obj_info in dynamic_objects:
            obj_pos = self.translate_sys_to_arena([obj_info[0], obj_info[1], 0.0])
            obj_size = self.translate_sys_to_arena([obj_info[2], obj_info[3], 0.0])
            arcade.draw_rectangle_filled(obj_pos[0], obj_pos[1], obj_size[0] - self.ZERO_BASE_X, obj_size[1] - self.ZERO_BASE_Y, arcade.color.RED)
            arcade.draw_rectangle_filled(obj_pos[0], obj_pos[1], 5, 5, arcade.color.BLACK)

        # draw specification set (safe/target)
        spec_set = dynamic_sets[1]
        spec_set_pos = self.translate_sys_to_arena([spec_set[0], spec_set[1], 0.0])
        spec_set_size = self.translate_sys_to_arena([spec_set[2], spec_set[3], 0.0])
        arcade.draw_rectangle_filled(spec_set_pos[0], spec_set_pos[1], spec_set_size[0] - self.ZERO_BASE_X, spec_set_size[1] - self.ZERO_BASE_Y, arcade.color.GREEN)

    def start(self):
        arcade.run()

    def get_current_symbol(self):
        return self.qnt_x.conc_to_flat(self.sys_state)
    
    def print_info(self):
        txt  = "Status: " + self.sys_status
        txt += " | Time (sec.): " + str(round(self.time_elapsed))
        txt += " | FPS: " + str(round(1/self.avg_delta))
        txt += "\nState: x_" + str(self.get_current_symbol()) + " = (" + list2str(self.sys_state) + ")"
        txt += " | Control action: "
        
        if self.last_action_symbol == -1:
            txt += "not_issued"
        else:
            txt += "u_" + str(self.last_action_symbol) + " (" + list2str(self.last_action) + ")"

        arcade.draw_text(txt, self.ZERO_BASE_X, self.ZERO_BASE_Y - 35, arcade.color.BLACK, 10)

    def on_draw(self):
        arcade.start_render()
        self.draw_arena()
        self.system.draw()
        self.print_info()

    def update(self, delta_time):
        self.time_elapsed += delta_time
        self.avg_delta = (self.avg_delta + delta_time)/2.0

