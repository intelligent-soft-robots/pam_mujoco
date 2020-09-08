import os
import itertools
import numpy as np

def _str(a):
    return " ".join([str(a_) for a_ in a])


class HittingPoint:
    
    def __init__(self,
                 name,
                 position=[0,0,-0.455],
                 size=[0.03,0.0007],
                 color=[1.0,0.65,0.1,1]):
        self.name = name
        self.position = position
        self.size = size
        self.color = color

    def get_xml(self):
        xml = ['<body pos = "'+_str(self.position)+'" name="'+self.name+'">',
               '<geom type="cylinder" size="'+_str(self.size)+'" rgba="'+str(self.color)+'"/>',
               '<joint type="slide" axis="1 0 0" name="'+self.name+'_x"/>',
               '<joint type="slide" axis="0 1 0" name="'+self.name+'_y"/>',
               '</body>']
        return xml


class Goal:

    def __init__(self,
                 name,
                 position,
                 radius1=0.05,
                 radius2=0.2,
                 color1=[0.2,1.0,0.2,1.0],
                 color2=[0.2,1.0,0.2,0.05]):
        self.name = name
        self.position = position
        self.radius1 = radius1
        self.radius2 = radius2
        self.color1 = color1
        self.color2 = color2
        
    def get_xml(self):
        xml = ['<body pos = "'+_str(self.position)+'" name="'+self.name+'">',
               '<geom name="'+self.name+'_inner" type="cylinder" pos="0 0 0" size="'+str(self.radius1)+' 0.0005" rgba="'+_str(self.color1)+'"/>',
               '<geom name="'+self.name+'_outer" type="cylinder" pos="0 0 0" size="'+str(self.radius2)+' 0.0004" rgba="'+_str(self.color2)+'"/>',
               '</body>']
        return xml

class Ball:

    def __init__(self,
                 name,
                 size=0.02,
                 color=[1.0,0.65,0.1,1.0],
                 position=[0,0,0],
                 mass=0.0027):
        self.name = name
        self.size = size
        self.color = color
        self.position = position
        self.mass = mass

    def get_xml(self):
        position = " ".join([str(p) for p in self.position])
        size = str(self.size)
        color = " ".join([str(c) for c in self.color])
        mass = str(self.mass)
        xml = ['<body pos="'+position+'" name="'+self.name+'">', 
               '<geom name="'+self.name+'" type="sphere" size="'+size+'" rgba="'+color
               +'" pos="'+position+'" mass="'+mass+'"/>',
               '<joint type="free" name="'+self.name+'"/>',
               '</body>']
        return xml

class Table:

    def __init__(self,
                 name,
                 position=[0.8,1.7,-0.475],
                 size=[0.7625,1.37,0.02],
                 color=[0.05,0.3,0.23,1.0]):
        self.name = name
        self.position = position
        self.size = size
        self.color = color

    def get_xml(self):
        position = " ".join([str(p) for p in self.position])
        position = '<body pos = "'+position+'" name = "'+self.name+'">'
        color = " ".join([str(c) for c in self.color])
        size = " ".join(str(s) for s in self.size) 
        plate =  '<geom name="'+self.name+'_plate" type="box" size="'+size+'" rgba="'+color+'"  zaxis="0 0 1"/>'
        legs = np.array([0.9*s for s in self.size[:2]])
        combs = np.array(list(itertools.product([-1,1],repeat=2)))
        legs = legs*combs
        legs_z = np.array([-0.38]*4).reshape(-1,1)
        legs = np.append(legs,legs_z,axis=1)
        def get_leg(index_position):
            index,position = index_position
            position = " ".join([str(p) for p in position])
            return '<geom name="leg'+str(index)+'" pos = "'+position+'" type="box" size="0.02 0.02 0.38" rgba="0.1 0.1 0.1 1.0"/>'
        legs = list(enumerate(legs))
        legs = list(map(get_leg,legs))
        net = '<geom name="'+self.name+'_net" pos = "0.0 0.0 0.07625" type="box" size="0.7825 0.02 0.07625" rgba="0.1 0.1 0.1 0.4"/>'
        body_end = '</body>'
        return [position]+[plate]+legs+[net]+[body_end]


def contacts_xml(robots,balls,
                 tables,dampings,gaps):
    
    def get_attrs(geom1,geom2):
        damping = None
        gap = None
        if geom1 in dampings :
            if geom2 in dampings[geom1]:
                damping = _str(dampings[geom1][geom2])
        if geom1 in gaps:
            if geom2 in gaps[geom1]:
                gap = str(gaps[geom1][geom2])
        r = ""
        if damping:
            r+= ' solref="'+damping+'" '
        if gap:
            r+= ' gap="'+gap+'"'
        r+="/>"
        return r
    balls = [ball.name for ball in balls]
    robots = list(robots.keys())
    tables = [table.name for table in tables]
    contacts = []
    for ball in balls:
        contacts.append('<pair geom1="'+ball+'" geom2="floor"'+get_attrs("ball",
                                                                        "floor"))
        for robot in robots:
            contacts.append('<pair geom1="'+ball+'" geom2="'+robot+'_racket"'+get_attrs("ball",
                                                                                       "racket"))
            contacts.append('<pair geom1="'+ball+'" geom2="'+robot+'_racket_handle"'+get_attrs("ball",
                                                                                              "racket_handle"))
        for table in tables:
            contacts.append('<pair geom1="'+ball+'" geom2="'+table+'_plate"'+get_attrs("ball",
                                                                                       "table"))
            contacts.append('<pair geom1="'+ball+'" geom2="'+table+'_net"'+get_attrs("ball",
                                                                                     "net"))
    for robot in robots:
        contacts.append('<pair geom1="floor" geom2="'+robot+'_racket"'+get_attrs("floor",
                                                                                "racket"))

    return contacts
        
def get_models_path():
    from catkin_pkg import workspaces
    packages = workspaces.get_spaces()
    context_pkg_path = [ p for p in packages
                         if p.endswith("pam_mujoco") ][0]
    return os.path.join(context_pkg_path,
                        "models")

def get_main_template_xml():
    path = get_models_path()+os.sep+"template.xml"
    with open(path,"r") as f:
        template = f.read()
    return template
    
def get_robot_templates_path():
    return get_models_path()+os.sep+"robot_templates"

def get_tmp_path():
    return get_models_path()+os.sep+"tmp"

def get_robot_xml(filename, ir):
    path = get_robot_templates_path()+os.sep+filename+".xml"
    with open(path,"r") as f:
        template = f.read()
    template = template.replace("?id?",str(ir))
    return template
    
def get_robot_body_xml(ir):
    return get_robot_xml("body_template",ir)

def get_robot_tendon_xml(ir):
    return get_robot_xml("tendon_template",ir)

def get_robot_actuator_xml(ir):
    return get_robot_xml("actuator_template",ir)

def write_robot_body_xml(ir):
    xml = get_robot_body_xml(ir)
    filename = "robot_body_"+str(ir)+".xml"
    with open(get_tmp_path()+os.sep+filename,"w+") as f:
        f.write(xml)
    return filename

def write_robot_tendon_xml(ir):
    xml = get_robot_tendon_xml(ir)
    filename = "robot_tendon_"+str(ir)+".xml"
    with open(get_tmp_path()+os.sep+filename,"w+") as f:
        f.write(xml)
    return filename

def write_robot_actuator_xml(ir):
    xml = get_robot_actuator_xml(ir)
    filename = "robot_actuator_"+str(ir)+".xml"
    with open(get_tmp_path()+os.sep+filename,"w+") as f:
        f.write(xml)
    return filename

def write_model_xml(model_name,xml_content):
    path = get_tmp_path()+os.sep+model_name+".xml"
    with open(path,"w+") as f:
        f.write(xml_content)
    return path

# robots : {robot_id:(position,xyaxes)}
def generate_model(model_name,
                   robots={},
                   balls=[],
                   tables=[],
                   goals=[],
                   hitting_points=[],
                   dampings={ "ball":{ "racket":(-0.1,-0.1),
                                       "floor":(0.003,0.25),
                                       "racket_handle":(0.003,0.35),
                                       "table":(-0.1,-0.1),
                                       "net":(0.003,10.0) },
                              "floor":{ "racket":(0.003,0.9) } },
                   gaps={"ball":{"floor":0.0,"table":0.02}}):

    template = get_main_template_xml()

    bodies = []
    for robot_id,attributes in robots.items():
        filename = write_robot_body_xml(robot_id)
        position = attributes[0]
        position = 'pos="'+" ".join([str(p) for p in position])+'"'
        xyaxes = ""
        if attributes[1] is not None:
            xyaxes = ' xyaxes="'+" ".join([str(a) for a in attributes[1]])+'"'
        xml = ['<body name="'+str(robot_id)+'" '+position+xyaxes+'>',
               '<include file="'+filename+'"/>'
               '</body>']
        bodies+=xml

    for ball in balls:
        bodies += ball.get_xml()

    for table in tables:
        bodies += table.get_xml()

    for goal in goals:
        bodies += goal.get_xml()

    for hitting_point in hitting_points:
        bodies += hitting_point.get_xml()
        
    template = template.replace("<!-- bodies -->","\n".join(bodies))

    actuations = ["<tendon>"]
    for robot_id in robots.keys():
        xml_tendon = get_robot_tendon_xml(robot_id)
        actuations.append(xml_tendon)
    actuations.append("</tendon>")
    actuations.append("<actuator>")
    for robot_id in robots.keys():
        xml_actuator = get_robot_actuator_xml(robot_id)
        actuations.append(xml_actuator)
    actuations.append("</actuator>")

    template = template.replace("<!-- actuations -->","\n".join(actuations))

    contacts = contacts_xml(robots,balls,tables,dampings,gaps)

    template = template.replace("<!-- contacts -->","\n".join(contacts))
    
    path = write_model_xml(model_name,template)

    return path
        
        

def model_factory(model_name,
                  table=False,nb_balls=1,robot1=False,
                  robot2=False,goal=False,hitting_point=False):

    tables = []
    if table :
        tables.append(Table("table"))

    balls = [Ball("ball_"+str(index))
             for index,ball in enumerate(range(nb_balls))]

    robots = {}
    if robot1:
        robots["robot1"]=([0,0,-0.44],None)
    if robot2:
        robots["robot2"]=([1.6,3.4,-0.44],[-1,0,0,0,-1,0])

    if goal:
        goals = [Goal("goal")]
    else:
        goals = []

    if hitting_point:
        hitting_points = [HittingPoint("hitting_point")]
    else:
        hitting_points = []

    return generate_model(model_name,
                          robots=robots,
                          balls=balls,
                          tables=tables,
                          goals=goals,
                          hitting_points=hitting_points)
