import os

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
def generate_model(model_name,robots):

    template = get_main_template_xml()

    bodies = []
    for robot_id,attributes in robots.items():
        filename = write_robot_body_xml(str(robot_id))
        position = attributes[0]
        position = 'pos="'+" ".join([str(p) for p in position])+'"'
        xyaxes = ""
        if attributes[1] is not None:
            xyaxes = ' xyaxes="'+" ".join([str(a) for a in attributes[1]])+'"'
        xml = ['<body name="'+str(robot_id)+'" '+position+xyaxes+'>',
               '<include file="'+filename+'"/>',
               '</body>']
        bodies+=xml

    template = template.replace("<!-- bodies -->","\n".join(bodies))

    actuations = ["<tendon>"]
    for robot_id in robots.keys():
        filename = write_robot_tendon_xml(robot_id)
        xml = '<include_file="'+filename+'"/>'
        actuations.append(xml)
    actuations.append("</tendon>")
    actuations.append("<actuator>")
    for robot_id in robots.keys():
        filename = write_robot_actuator_xml(robot_id)
        xml = '<include_file="'+filename+'"/>'
        actuations.append(xml)
    actuations.append("</actuator>")

    template = template.replace("<!-- actuations -->","\n".join(actuations))

    path = write_model_xml(model_name,template)

    return path
        
        

    
