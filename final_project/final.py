import random
import time
import numpy
import copy
import math
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

    def delete_node(self, node):
        if node in self.graph:
            del self.graph[node]
            for n, neighbors in self.graph.items():
                if node in neighbors:
                    neighbors.remove(node)

    def delete_edge(self, node1, node2):
        if node1 in self.graph and node2 in self.graph[node1]:
            self.graph[node1].remove(node2)
    
    def get_nodes(self):
        lst = []
        for node in self.graph:
            if node != 'body':
                lst.append(node)
        return(lst)
    
    def get_edges(self, node):
        return self.graph[node]
    
    def make_copy(self):
        return copy.deepcopy(self)
    
    def get_graph(self):
        for node, neighbors in self.graph.items():
            print(f"{node}: {neighbors}")



# simple func to quickly choose a PRETTY rgba color 
def choose_color():
    pink = [1.4, 0.5, 0.75, 1.0]
    blue = [0.4, 0.7, 1.5, 1.0]
    orange = [1.35, 0.9, 0.3, 1.0]
    return ' '.join(map(str, random.choice([pink, blue, orange])))

# randomly chooses a shape and generates suitable sizes given a range
def get_shape_and_size(range1, range2, part):
    shape = random.choice(['box', 'sphere']) if part == 'body' else random.choice(['capsule', 'cylinder'])

    if shape in ['capsule', 'cylinder']:
        shape_xz = round(random.uniform(range1[0], range1[1]), 2)
        shape_y = round(random.uniform(range2[0], range2[1]), 2)
        return shape, [shape_xz, shape_y, shape_y] 
    if shape in ['sphere', 'box']:
        shape_xyz = round(random.uniform(range1[0], range1[1]), 2)
        return shape, [shape_xyz, shape_xyz, shape_xyz]

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
            z_euler = rotate_top
            return ([z_euler[1], 0, 0], [0, z_euler[0], 0])
    
    else:
        return ([0, 0, -leg_size[1]*2], [0, 0, 0])
    
# generates suitable pos/axis for joints based on leg size
def get_joint_pos_axis_range(leg_size):
    joint_range = random.randint(30, 80)
    return ([0, 0, leg_size[1]], [0, 90, 0], [-joint_range, joint_range])



# fills in xml template for body with given info
def build_body(body, name, pos, euler):
    body.set('name', name)
    body.set('pos', ' '.join(map(str, pos)))
    body.set('euler', ' '.join(map(str, euler)))
    return body

def build_joint(joint, name, joint_type, pos, axis, joint_range):
    if joint_type != 'free':
        joint.set('name', name)
        joint.set('type', joint_type)
        joint.set('pos', ' '.join(map(str, pos)))
        joint.set('axis', ' '.join(map(str, axis)))
        joint.set('range', ' '.join(map(str, joint_range)))
    else:
        joint.set('type', joint_type)
    return joint

def build_geom(geom, geom_type, size, rgba):
    geom.set('type', geom_type)
    geom.set('size', ' '.join(map(str, size)))
    geom.set('rgba', rgba)
    return geom

def build_actuator(joint):
    motor = ET.Element('motor')
    motor.set('gear', '100')
    motor.set('joint', joint)
    return motor

# assembles all pieces of the body
def build_unit(unit, name, body_info, joint_info, geom_info):
    build_body(unit, name, body_info[0], body_info[1])
    joint = ET.SubElement(unit, 'joint')
    build_joint(joint, name + '_joint', 'hinge', joint_info[0], joint_info[1], joint_info[2])
    geom = ET.SubElement(unit, 'geom')
    build_geom(geom, geom_info[0], geom_info[1], geom_info[2])

# creates the motors for main leg joints
def build_creature_motors(geno):
    actuators = ET.Element('actuator')
    for node in geno.graph.items():
        if node[0] != 'body':
            actuator = build_actuator(node[0] + '_joint')  # Generate individual motor
            actuators.append(actuator)
    return actuators



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

# generate pheno traits for building creature --> helps track for mutations
def generate_pheno_traits(geno):
    body_shape_size = get_shape_and_size([0.25, 0.3], [0.35, 0.4], 'body')
    leg_shape_size = get_shape_and_size([0.1, 0.15], [0.25, 0.3], 'leg')
    leg_positions_euler = []
    for unit in geno.graph.items():
        if unit[0] != 'body':
            leg_positions_euler.append(get_pos_and_euler(body_shape_size[1], leg_shape_size[1], True))
    return [body_shape_size[0], body_shape_size[1], leg_shape_size[0], leg_shape_size[1], leg_positions_euler]

# creates the xml phenotype of a creature based on the geno type
def build_creature_pheno(geno, body_shape, body_size, leg_shape, leg_size, leg_positions, leg_color, body_color):
    leg_pos_counter = 0
    for unit in geno.graph.items():
        if unit[0] == 'body':
            main_body = ET.Element('body', name='mainbody', pos='0 0 0.5', euler='0 90 0')
            joint = ET.SubElement(main_body, 'joint', type='free')
            geom = ET.SubElement(main_body, 'geom', type=body_shape, size=' '.join(map(str, body_size)), rgba=body_color)
        else:
            joint_info = get_joint_pos_axis_range(leg_size)
            geom_info = [leg_shape, leg_size, leg_color]
            leg = ET.SubElement(main_body, 'body')
            build_unit(leg, unit[0], leg_positions[leg_pos_counter], joint_info, geom_info)
            leg_pos_counter += 1
            seg_counter = 0
            for seg in unit[1]:
                seg_info = get_pos_and_euler(body_size, leg_size, False)
                if seg_counter == 0:
                    segment = ET.SubElement(leg, 'body')
                else: 
                     segment = ET.SubElement(segment, 'body')
                build_unit(segment, seg + '_seg' + str(seg_counter), seg_info, joint_info, geom_info)
                seg_counter += 1
    return main_body

# build mujoco file
def build_world(creature, motors, xml_file):
    # build initial mujoco world with nothing 
    mujoco_world = ET.Element('mujoco')
    option = ET.SubElement(mujoco_world, 'option', gravity='0 0 -50')
    worldbody = ET.SubElement(mujoco_world, 'worldbody')
    
    # insert creature body and actuators into place
    find_location = mujoco_world.find(".//worldbody")
    find_location.insert(0, creature)
    find_location.insert(0, ET.Element('geom', type='plane', size='5 5 0.1', rgba='0.8 0.8 0.8 1'))
    find_location.insert(0, ET.Element('light', diffuse='0.5 0.5 0.6', pos='0 2 3', dir='0 -0.5 -1'))
    mujoco_world.append(motors)
    tree = ET.ElementTree(mujoco_world)
    tree.write(xml_file)



# modifying geno randomly
def mutate_geno(parent_geno):
    leg_seg = random.randint(0, 1)
    if len(parent_geno.get_nodes()) <= 1 or len(parent_geno.get_edges('leg0')) == 0:
        del_add = 1
    else:
        del_add = random.randint(0, 1)
    new_geno = parent_geno.make_copy()
    leg_num = 'leg' + str(len(parent_geno.get_nodes()) - 1)
    if leg_seg == 0:
        if del_add == 0:
            new_geno.delete_node(leg_num)
        if del_add == 1:
            new_leg = 'leg' + str(len(parent_geno.get_nodes()))
            new_geno.add_node(new_leg)
            new_geno.add_edge('body', new_leg)
            for i in new_geno.get_edges('leg0'):
                new_geno.add_edge(new_leg, new_leg)
    else: 
        if del_add == 0:
            for i in new_geno.get_nodes():
                if new_geno.get_edges(i) == []:
                    new_geno.delete_node(i)
                else:
                    new_geno.delete_edge(i, i)
        if del_add == 1:
            for i in new_geno.get_nodes():
                new_geno.add_edge(i, i)
    return new_geno

# modifying pheno randomly    
def mutate_pheno(parent_pheno, mutated_geno):
    new_pheno = copy.deepcopy(parent_pheno)
    if len(mutated_geno.get_nodes()) > len(parent_pheno[4]):
        new_pheno[4].append(get_pos_and_euler(parent_pheno[1], parent_pheno[3], True))
    if len(mutated_geno.get_nodes()) < len(parent_pheno[4]):
        new_pheno[4].pop(len(parent_pheno[4]) - 1)
    mutated_pheno = random.randint(0, len(parent_pheno) - 1)
    if mutated_pheno == 0:
        new_pheno[mutated_pheno] = random.choice(['box', 'sphere'])
    elif mutated_pheno == 2:
        new_pheno[mutated_pheno] = random.choice(['capsule', 'cylinder'])
    elif mutated_pheno == 4:
        pass
    else:
        change = round(random.uniform(-0.06, 0.06), 2)
        for i in range(len(new_pheno[mutated_pheno])): 
            new_pheno[mutated_pheno][i] += change
            if new_pheno[mutated_pheno][i] <= 0.001:
                new_pheno[mutated_pheno][i] = 0.01
    return new_pheno



# helper for recording simulations    
def record_pos(pos):
    lst = []
    for i in pos:
        lst.append(i)
    return lst

# sorting func for sorting list of fitness records
def sort_func(lst):
    return lst[2]

### main simulations ###
def run_initial_simulation(num_parents):
    fitness_record = []
    for j in range(num_parents):
        # actually simulating the model:
        num_leg = random.randint(1, 8)
        geno = init_creature_geno(num_leg, random.randint(0, 3))
        pheno_traits = generate_pheno_traits(geno)
        pheno = build_creature_pheno(geno, pheno_traits[0], pheno_traits[1], pheno_traits[2], pheno_traits[3], pheno_traits[4], choose_color(), choose_color())
        run_fitness = [geno, pheno_traits]
        build_world(pheno, build_creature_motors(geno), 'final.xml')
        new_arr = [45] * num_leg
        
        m = dm_control.mujoco.MjModel.from_xml_path("final.xml")
        d = dm_control.mujoco.MjData(m)
        viewer = False
        viewer = mujoco.viewer.launch_passive(m, d)
        viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
        viewer.cam.elevation = -50  # Elevation angle (in degrees)
        viewer.cam.distance = 15.0

        actuators = m.nu
        wiggle = numpy.array(new_arr)
        d.ctrl[:actuators] = wiggle
        start = record_pos(d.body('mainbody').xpos)
        for i in range(200):
            if viewer.is_running():
                if i%25 == 0:
                    wiggle = -wiggle
                d.ctrl[:actuators] = wiggle 
                mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
            else:
                break
        end = d.body('mainbody').xpos
        record = math.dist(start, end)
        print(record)
        run_fitness.append(record)
        fitness_record.append(run_fitness)
        viewer.close()
        time.sleep(0.05)
    fitness_record.sort(key = sort_func, reverse = True)
    winner = fitness_record[0]
    print('\nwinner: ', winner[2])
    winner[0].get_graph()
    new_fitness = fitness_record[0:len(fitness_record)//2] 
    for i in new_fitness:
        print(i[2])
    return(new_fitness)

def begin_evolution(fitness_record, counter, evolve_num):
    if counter >= evolve_num:
        print('\n\nfinal geno:')
        fitness_record[0][0].get_graph()
        print('final pheno:', fitness_record[0][1])
        print('final fitness:', fitness_record[0][2])
        return
    counter += 1
    print('\n\ngeneration:', counter)
    for j in range(len(fitness_record)):
        
        # actually simulating the model:
        mutated_geno = mutate_geno(fitness_record[j][0])
        mutated_pheno = mutate_pheno(fitness_record[j][1], mutated_geno)
        child = build_creature_pheno(mutated_geno, mutated_pheno[0], mutated_pheno[1], mutated_pheno[2], mutated_pheno[3], mutated_pheno[4], choose_color(), choose_color())
        run_fitness = [mutated_geno, mutated_pheno]
        build_world(child, build_creature_motors(mutated_geno), 'final.xml')
        new_arr = [45] * len(mutated_geno.get_nodes())
        m = dm_control.mujoco.MjModel.from_xml_path("final.xml")
        d = dm_control.mujoco.MjData(m)
        viewer = mujoco.viewer.launch_passive(m, d)
        viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
        viewer.cam.elevation = -50  # Elevation angle (in degrees)
        viewer.cam.distance = 15.0

        actuators = m.nu
        wiggle = numpy.array(new_arr)
        d.ctrl[:actuators] = wiggle
        start = record_pos(d.body('mainbody').xpos)
    
        for i in range(200):
            if viewer.is_running():
                if i%25 == 0:
                    wiggle = -wiggle
                d.ctrl[:actuators] = wiggle 
                mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)
            else:
                break
        end = d.body('mainbody').xpos
        record = math.dist(start, end)
        print('mutated from', j, ':', record)
        run_fitness.append(record)
        fitness_record.append(run_fitness)
        viewer.close()
        time.sleep(0.2)
    fitness_record.sort(key = sort_func, reverse = True)
    winner = fitness_record[0]
    print('\nwinner: ', winner[2])
    winner[0].get_graph()
    new_fitness = fitness_record[0:len(fitness_record)//2] 
    print('\ncurrent records:')
    for i in new_fitness:
        print(i[2])
    begin_evolution(new_fitness, counter, evolve_num)



fitness = run_initial_simulation(30)
print('\nbeginning evolution...\n')
begin_evolution(fitness, 0, 10)
