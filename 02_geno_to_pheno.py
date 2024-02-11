import random
import time
import numpy
import xml.etree.ElementTree as ET
import dm_control.mujoco
import mujoco.viewer

# create genotype graph
class Graph:
    def __init__(self):
        self.graph = {}

    def add_node(self, node):
        if node not in self.graph:
            self.graph[node] = []

    def add_edge(self, node1, node2):
        if node1 in self.graph and node2 in self.graph:
            self.graph[node1].append(node2)

    def get_graph(self):
        for node, neighbors in self.graph.items():
            print(f"{node}: {neighbors}")

### helper functions ###    
# simple func to quickly convert info into xml string input format
def convert_to_string(info_list):
    return ' '.join(map(str, info_list))
# simple func to quickly reformats size shorthand for xml template
def reformat_sizes(shape, size):
    if shape == 'box':
        return [size[0], size[0], size[1]]
    elif shape == 'cylinder' or shape == 'capsule':
        return [size[0], size[1], size[1]]
    elif shape == 'sphere':
        return [size[0], size[0], size[0]]
# simple funct to quickly choose a PRETTY rgba color 
def choose_color():
    pink = [1.4, 0.5, 0.75, 1.0]
    blue = [0.4, 0.7, 1.5, 1.0]
    orange = [1.35, 0.9, 0.3, 1.0]
    return convert_to_string(random.choice([pink, blue, orange]))

### getter functions for building parts of the creature ###
# randomly chooses a shape and generates suitable sizes given a range
def get_shape_and_size(range1, range2, part):
    shape = random.choice(['box', 'sphere']) if part == 'body' else random.choice(['box', 'capsule', 'cylinder'])

    if shape in ['box', 'capsule', 'cylinder']:
        shape_xz = round(random.uniform(range1[0], range1[1]), 2)
        shape_y = round(random.uniform(range2[0], range2[1]), 2)
        return shape, [shape_xz, shape_xz] if part == 'body' else [shape_xz, shape_y]
    elif shape == 'sphere':
        shape_xyz = round(random.uniform(range1[0], range1[1]), 2)
        return shape, [shape_xyz, shape_xyz]
# generates suitable pos/euler for legs based on body/leg sizes
def get_pos_and_euler(body_size, leg_size, first):

    if first == True:
        rotate_0 = [0, -(body_size[1] + leg_size[1])]
        rotate_90 = [90, body_size[0] + leg_size[1]]
        rotate_180 = [180, body_size[1] + leg_size[1]]
        rotate_neg90 = [-90, -(body_size[0] + leg_size[1])]
        rotate_top = [90, -(body_size[0] + leg_size[1])]
        rotate_bottom = [-90, body_size[0] + leg_size[1]]
        choice = random.choice(['xy_euler','z_euler'])
        
        if choice == 'xy_euler':
            xy_euler = random.choice([rotate_0, rotate_90, rotate_180, rotate_neg90])
            if xy_euler in [rotate_neg90, rotate_90]:
                return ([0, xy_euler[1], 0], [xy_euler[0], 0, 0])
            elif xy_euler in [rotate_0, rotate_180]:
                return ([0, 0, xy_euler[1]], [xy_euler[0], 0, 0])
        elif choice == 'z_euler':
            z_euler = random.choice([rotate_top, rotate_bottom])
            return ([z_euler[1], 0, 0], [0, z_euler[0], 0])
    
    else:
        return ([0, 0, -leg_size[1]*2], [0, 0, 0])
    
# generates suitable pos/axis for joints based on leg size
def get_joint_pos_axis_range(leg_size):
    joint_range = random.randint(30, 80)
    return ([0, 0, leg_size[1]], [0, 90, 0], [-joint_range, joint_range])

### adder functions for building the xml file
# fills in xml template for body with given info
def add_body(body_shape, body_size):
    body = '''<body name='body' pos='0 0 1' euler='0 90 0'>{}{}{}\n</body>'''
    joint = '''\n\t<joint type='free'/>'''
    geom = '''\n\t<geom type='{}' size='{}' rgba='{}'/>'''.format(body_shape, convert_to_string(reformat_sizes(body_shape, body_size)), choose_color(), {})
    return body.format(joint, geom, {})

# fills in xml template for leg with given info
def add_leg(leg_name, leg_shape, leg_size, leg_pos_euler, joint_pos_axis_range, end):
    geom_temp = '''\n\t\t<geom type='{}' size='{}' rgba='{}'/>{}'''
    if end == False:
        geom = geom_temp.format(leg_shape, convert_to_string(reformat_sizes(leg_shape, leg_size)), choose_color(), {})
    else:
        geom = geom_temp.format(leg_shape, convert_to_string(reformat_sizes(leg_shape, leg_size)), choose_color(), '')
    body = '''\n\t<body name='{}' pos='{}' euler='{}'>{}{}\n\t</body>'''.format(leg_name, convert_to_string(leg_pos_euler[0]), convert_to_string(leg_pos_euler[1]), {}, {})
    joint = '''\n\t\t<joint name ='{}' type='hinge' pos='{}' axis='{}' range='{}'/>'''.format(leg_name + '_joint', convert_to_string(joint_pos_axis_range[0]), convert_to_string(joint_pos_axis_range[1]), convert_to_string(joint_pos_axis_range[2]))
    return body.format(joint, geom)
# fills in xml template for actuator with joint name
def add_actuator(joint):
    return '''<motor gear='{}' joint='{}'/>\n'''.format(random.randint(30, 80), joint)


###  builds the geno and pheno, moves to xml ###
# initialize the creature graph based on the above parameters
def init_creature_geno(leg_num, leg_seg_num):
    creature = Graph()
    creature.add_node('body')
    for i in range(leg_num):
        which_leg = 'leg' + str(i)
        creature.add_node(which_leg)
        creature.add_edge('body', which_leg)
        for i in range(leg_seg_num):
            creature.add_edge(which_leg, which_leg)
    return creature
# creates the xml phenotype of a creature based on the geno type
def build_creature_pheno(geno):
    body_shape_size = get_shape_and_size([0.2, 0.25], [0.25, 0.3], 'body')
    leg_shape_size = get_shape_and_size([0.06, 0.09], [0.1, 0.2], 'leg')
    pheno = '''{}'''
    legs = ''
    for node in geno.graph.items():
        if node[0] == 'body':
            body = add_body(body_shape_size[0], body_shape_size[1])
            pheno = pheno.format(body)
        else:
            leg_pos_euler = get_pos_and_euler(body_shape_size[1], leg_shape_size[1], True)
            joint_pos_axis_range = get_joint_pos_axis_range(leg_shape_size[1])
            leg = add_leg(node[0], leg_shape_size[0], leg_shape_size[1], leg_pos_euler, joint_pos_axis_range, False)
            
            segment_tracker = 0
            for neighbor in node[1]:
                segment_tracker += 1
                leg_pos_euler = get_pos_and_euler(body_shape_size[1], leg_shape_size[1], False)
                if segment_tracker == len(node[1]):
                    leg = leg.format(add_leg(neighbor + '_seg' + str(segment_tracker), leg_shape_size[0], leg_shape_size[1], leg_pos_euler, joint_pos_axis_range, True))
                else:
                    leg = leg.format(add_leg(neighbor + '_seg' + str(segment_tracker), leg_shape_size[0], leg_shape_size[1], leg_pos_euler, joint_pos_axis_range, False))
            legs = legs + leg
    pheno = pheno.format(legs)
    return pheno
# creates the motors for main leg joints
def build_creature_motors(geno):
    actuators = ''
    for node in geno.graph.items():
        if node[0] != 'body':
            actuators = actuators + add_actuator(node[0] + '_joint')
    return actuators
# writes all the information to an xml file
def write_to_xml(xml_file, geno):
    mujoco_world = '''
        <mujoco>
            <option gravity="0 0 -25"/>
            <worldbody>
                <light diffuse=".5 .5 .6" pos="0 2 3" dir="0 -0.5 -1"/>
                <geom type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
                {}
            </worldbody>
                <actuator>
                    {}
                </actuator>
        </mujoco>
    '''
    new_world = mujoco_world.format(build_creature_pheno(geno), build_creature_motors(geno))
    root = ET.XML(new_world)
    tree = ET.ElementTree(root)
    tree.write(xml_file)

### main simulation ###
def main(num_leg, num_seg):
    # actually simulating the model:
    geno = init_creature_geno(num_leg, num_seg)
    geno.get_graph()
    write_to_xml('blah.xml', geno)
    new_arr = random.sample(range(-50, 50), num_leg)

    m = dm_control.mujoco.MjModel.from_xml_path("blah.xml")
    d = dm_control.mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        actuators = m.nu
        wiggle = numpy.array(new_arr)
        d.ctrl[:actuators] = wiggle

        for i in range(10000):
            if viewer.is_running():
                if i%30 == 0:
                    wiggle = -(wiggle+10)
                d.ctrl[:actuators] = wiggle 
                mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
            else:
                break
        viewer.close()

main(random.randint(3, 6), random.randint(1, 2))