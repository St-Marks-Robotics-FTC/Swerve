import pygame, time, math
from pygame.locals import *

WINDOW_WIDTH = 600
WINDOW_HEIGHT = 600
MODULE_WIDTH = 20
MODULE_HEIGHT = 40
assert(WINDOW_WIDTH%3==0 and WINDOW_HEIGHT%3==0, "Window width and height must be divisible by 3.")
# Colors
#             R    G    B
BROWN     = ( 82,  70,  70)
GREEN     = ( 74, 140,  74)
BLUE      = ( 93,  71, 173)
WHITE     = (255, 255, 255)

FPS = 40

SWERVE_SIMULATION_ANGULAR_TOP_SPEED = 1.0 #deg/period
MOTOR_A_ROTATION_ANGLE = 180.0 + 45.0
MOTOR_B_ROTATION_ANGLE = 90.0 + 45.0
MOTOR_C_ROTATION_ANGLE = 0.0 + 45.0
MOTOR_D_ROTATION_ANGLE = 270.0 + 45.0


def main():
    
    pygame.init()
    screen_dimensions = (WINDOW_WIDTH, WINDOW_HEIGHT)
    
    # create the window
    screen_dimensions = ((WINDOW_WIDTH, WINDOW_HEIGHT))
    screen = pygame.display.set_mode(screen_dimensions)
    pygame.display.set_caption('Swerve Drive Simulation')

    # initialise the fps clock to regulate the fps
    fps_clock = pygame.time.Clock()

    # create an instance of the Game() class
    game = Game()
    
    
    while True:
        # process events eg keystrokes etc.
        #game.process_events()

        # run the game logic and check for collisions
        game.logic()

        # render the player and projectiles or the game over screen
        game.render(screen)

        # regulate the game fps
        fps_clock.tick(FPS)

class Game(object):
    
    def __init__(self):
        self.swerve_a_position = (WINDOW_WIDTH/3, WINDOW_HEIGHT/3)
        self.swerve_b_position = (WINDOW_WIDTH/3, WINDOW_HEIGHT/3*2)
        self.swerve_c_position = (WINDOW_WIDTH/3*2, WINDOW_HEIGHT/3*2)
        self.swerve_d_position = (WINDOW_WIDTH/3*2, WINDOW_HEIGHT/3)
        self.a_sprite = ModuleSprite(self.swerve_a_position)
        self.b_sprite = ModuleSprite(self.swerve_b_position)
        self.c_sprite = ModuleSprite(self.swerve_c_position)
        self.d_sprite = ModuleSprite(self.swerve_d_position)
        self.swerve_sprites = [self.a_sprite, self.b_sprite, self.c_sprite, self.d_sprite]
        
        self.swerve = SwerveDrive()
        self.swerve.drive(0.0, 0.0, 1.0, 1.0)
        
    def logic(self):
        for module, module_sprite in zip(self.swerve.swerve_drives, self.swerve_sprites):
            module.update_position()
            module_sprite.update_position(module.current_swerve_angle, module.current_swerve_speed)
    
    def render(self, screen):
        screen.fill(BLUE)
        
        for module_sprite in self.swerve_sprites:
            screen.blit(module_sprite.image, module_sprite.rect)
        
        pygame.display.flip()

class ModuleSprite(pygame.sprite.Sprite):
    def __init__(self, center):
        pygame.sprite.Sprite.__init__(self)
        self.image_master = pygame.Surface((MODULE_WIDTH, MODULE_HEIGHT))
        self.image_master.fill((255/2, 255,2, 255/2))
        self.image = self.image_master
        self.rect = self.image.get_rect()
        self.center = center
        self.rect.center = self.center
    
    def update_position(self, angle, speed):
        #angle (0 to 360), speed (-1 to 1)
        greyscale_value = (speed+1)/2*255 #scale speed from 0 to 255
        #reset the image
        self.image = self.image_master
        self.image.fill((greyscale_value, greyscale_value, greyscale_value))
        self.image = pygame.transform.rotate(self.image, -angle) #rotate the rectangle to the same angle as the swerve drive
        """self.rect = self.image.get_rect()
        self.rect.center = self.center"""

class SwerveDrive(object):
    """Swerve module names:
        A(0)----------D(3)
        |      ^      |
        |    / | \    |
        |      |      |
        |             |
        B(1)----------C(2) """
    def __init__(self):
        self.swerve_drives = []
        for x in range(4):
            self.swerve_drives.append(SwerveModule())
    
    def drive(self, vX, vY, vZ, throttle):
        #vX is the vertical axis and vY is the horizontal axis, so eqations for them are swapped relative to what you would think they are
        #                 x, y in standard cartesian coordinates, y, x in ours
        strafe_vector = [vY, vX]
        # add strafe vectors to rotational vectors, the cartesian coordinates of which are generated here
        mA = ((strafe_vector[1] + vZ * math.sin(math.radians(MOTOR_A_ROTATION_ANGLE))), (strafe_vector[0] + vZ * math.cos(math.radians(MOTOR_A_ROTATION_ANGLE))))
        mB = ((strafe_vector[1] + vZ * math.sin(math.radians(MOTOR_B_ROTATION_ANGLE))), (strafe_vector[0] + vZ * math.cos(math.radians(MOTOR_B_ROTATION_ANGLE))))
        mC = ((strafe_vector[1] + vZ * math.sin(math.radians(MOTOR_C_ROTATION_ANGLE))), (strafe_vector[0] + vZ * math.cos(math.radians(MOTOR_C_ROTATION_ANGLE))))
        mD = ((strafe_vector[1] + vZ * math.sin(math.radians(MOTOR_D_ROTATION_ANGLE))), (strafe_vector[0] + vZ * math.cos(math.radians(MOTOR_D_ROTATION_ANGLE))))
        motor_cartesian = (mA, mB, mC, mD)
        polar = []
        #x = 0
        #convert to polar and drive motors
        for cartesian in motor_cartesian:
            # angle, hypotenuse
            vector = [math.degrees(math.atan2(cartesian[1], cartesian[0])), math.sqrt(cartesian[0]**2 + cartesian[1]**2)]
            if vector[0] > 360:
                vector[0] = vector[0] % 360
            elif vector[0] < 0:
                vector[0] = 360 + (360 % vector[0])
            polar.append(vector)
            #print "Motor Number: ", x, " Angle: ", polar[-1][0], " Hypotenuse: ", polar[-1][1]
            #x+=1
        
        max_motor = 1.0
        for motor in polar:
            if motor[1] > max_motor:
                max_motor = motor[1]
        for motor in polar:
            motor[1]/=max_motor
            motor[1]*=throttle
        for module, vector in zip(self.swerve_drives, polar):
            module.set_direction_vector(vector)

class SwerveModule(object):
    def __init__(self):
        self._direction_vector = [0.0, 0.0] #[direction, speed] direction is 0 to 360 going clockwise and speed is 0 to 1
        self.current_swerve_angle = 0.0 #facing forwards
        self.current_swerve_speed = 0.0
        self.angle_desired = 0.0 # 0 to 360
        self.speed_desired = 0.0 # -1 to 1
    
    def set_direction_vector(self, _direction_vector):
        if 0.0 <= _direction_vector[0] <= 360.0 and 0.0 <= _direction_vector[1] <= 1.0:
            self._direction_vector = _direction_vector
            opposite_desired = (self._direction_vector[0] + 180.0) % 360.0
            opposite_forward = (self.current_swerve_angle + 180.0) % 360.0
            if abs(self.current_swerve_angle - self._direction_vector[0]) < abs(opposite_forward - self._direction_vector[0]):
                self.angle_desired = self._direction_vector[0]
                self.speed_desired = self._direction_vector[1]
            else:
                self.angle_desried = opposite_desired
                self.speed_desired = -self._direction_vector[1]
                
        else:
            print "SwerveModule.set_direction_vector: Must pass in a vector in the form (direction, hypotenuse) where direction is in range 0 to 360 and hypotenuse is in range 0 to 1."
    
    def update_position(self):
        #function to update the position and speed of the swerve drive so that it spins to the correct orientation then drives
        opposite_desired = (self._direction_vector[0] + 180.0) % 360.0
        if self.current_swerve_angle != self._direction_vector[0] and self.current_swerve_angle != opposite_desired:
            print "updating angle"
            #if the swerve drive is not currently in the correct position
            opposite_forward = (self.current_swerve_angle + 180.0) % 360.0
            if abs(self.current_swerve_angle - self._direction_vector[0]) < abs(opposite_forward - self._direction_vector[0]):
                difference = self.current_swerve_angle - self._direction_vector[0]
                #we want to drive the motor forwards, so spin the motor to the specified direction
                
                #figure out which direction we want the motor to move, and how far
                #in the real code, this could be replaced by a P-loop
                if difference > 0.0:
                    if difference >= 1.0:
                        self.current_swerve_angle -= SWERVE_SIMULATION_ANGULAR_TOP_SPEED
                    else:
                        self.current_swerve_angle = self._direction_vector[0]
                else:
                    if difference <= -1.0:
                        self.current_swerve_angle += SWERVE_SIMULATION_ANGULAR_TOP_SPEED
                    else:
                        self.current_swerve_angle = self._direction_vector[0]
            else:
                difference = opposite_forward - self._direction_vector[0]
                if difference > 0.0:
                    if difference >= 1.0:
                        self.current_swerve_angle -= SWERVE_SIMULATION_ANGULAR_TOP_SPEED
                    else:
                        self.current_swerve_angle = opposite_desired
                else:
                    if difference <= -1.0:
                        self.current_swerve_angle += SWERVE_SIMULATION_ANGULAR_TOP_SPEED
                    else:
                        self.current_swerve_angle = opposite_desired
        else:
            if self.current_swerve_angle == self._direction_vector[0]:
                self.current_swerve_speed = self._direction_vector[1]
            else:
                self.current_swerve_speed = -self._direction_vector[1]
        #wrap swerve ange from 0 to 360
        if self.current_swerve_angle > 360:
            self.current_swerve_angle = self.current_swerve_angle % 360
        elif self.current_swerve_angle < 0:
            self.current_swerve_angle =  360 + self.current_swerve_angle

if __name__ == "__main__":
    main()
