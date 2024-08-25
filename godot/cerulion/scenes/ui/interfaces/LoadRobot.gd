extends Control

@onready var window_text:Label = $VBoxContainer/Panel/MarginContainer/VBoxContainer/Label
var filepath:String = ""
var xml_parser:XMLParser = XMLParser.new()
var USER_DIR:DirAccess = DirAccess.open("user://")
var urdf_dir:String = ""
var urdf_name:String = ""

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	get_viewport().files_dropped.connect(onFilesDropped)
	if (not USER_DIR.dir_exists("robots")):
		USER_DIR.make_dir("robots")

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	pass

func onFilesDropped(files):
	parseFiles(files)
	parseURDF(RobotParameters.urdf_filepath)
	
func parseFiles(files):
	filepath = files[0]
	if (filepath.get_extension() != "zip"):
		window_text.text = "File must be .zip!"
	else:
		window_text.text = "Loaded file: " + filepath
		var zip_reader = ZIPReader.new()
		zip_reader.open(filepath)
		var zip_files = zip_reader.get_files()
		zip_files.sort()
		for f in zip_reader.get_files():
			if (f.ends_with(".urdf") and not f.begins_with("_")):
				urdf_name = (f.get_file()).trim_suffix(".urdf")
		USER_DIR.make_dir_recursive("robots/" + urdf_name)
		urdf_dir = "user://robots/" + urdf_name + "/"
		RobotParameters.urdf_dir = urdf_dir
		RobotParameters.urdf_filepath = RobotParameters.urdf_dir + urdf_name + ".urdf"
		print("Files in zip: ")
		for f in zip_reader.get_files():
			if (not f.begins_with("_")):
				print("	", f)
				if (f.ends_with("/")):
					USER_DIR.make_dir_recursive("robots/" + urdf_name + "/" + f)
				else:
					var write_file = FileAccess.open(urdf_dir + f, FileAccess.WRITE)
					var file_contents = zip_reader.read_file(f)
					write_file.store_buffer(file_contents)

func parseURDF(urdf_file):
	var parser = XMLParser.new()
	parser.open(urdf_file)
	var current_link:String = ""
	var current_joint:String = ""
	var current_attribute = ""
	var current_subattribute = ""
	var is_joint = false
	while parser.read() != ERR_FILE_EOF:
		if parser.get_node_type() == XMLParser.NODE_ELEMENT:
			#print("START" + parser.get_node_name())
			var node_name = parser.get_node_name()
			var link_dict:Dictionary = {
				"inertial": {
					"origin": {"xyz": [], "rpy": []},
					"mass": [],
					"inertia": {"ixx": [], "ixy": [], "ixz": [],
								"iyy": [], "iyz": [], "izz": []}, },
				"visual": {
					"origin": {"xyz": [], "rpy": []},
					"geometry": {"mesh": [], "obj": [], "mtl": []},
					"material": {"name": [], "color": []}, },
				"collision": {
					"origin": {"xyz": [], "rpy": []},
					"geometry": {}, }
			}
			var joint_dict:Dictionary = {
				"type": [],
				"dont_collapse": [],
				"origin": {"xyz": [], "rpy": []},
				"parent": [],
				"child": [],
				"axis": [],
				"limit": {"lower": [], "upper": [], "effort": [], "velocity": []}
			}
			var attributes_dict = {}
			for idx in range(parser.get_attribute_count()):
				attributes_dict[parser.get_attribute_name(idx)] = parser.get_attribute_value(idx)
			#print("The ", node_name, " element has the following attributes: ", attributes_dict)
			match node_name:
				"robot":
					RobotParameters.robot_name = attributes_dict["name"]
				"joint":
					current_joint = attributes_dict["name"]
					RobotParameters.joints[current_joint] = joint_dict.duplicate(true)
					RobotParameters.joints[current_joint]["type"] = attributes_dict["type"]
					if "dont_collapse" in attributes_dict.keys():
						RobotParameters.joints[current_joint]["dont_collapse"] = true \
							if (attributes_dict["dont_collapse"] == "true") else false
					is_joint = true
				"parent":
					RobotParameters.joints[current_joint]["parent"] = attributes_dict["link"]
				"child":
					RobotParameters.joints[current_joint]["child"] = attributes_dict["link"]
				"axis":
					RobotParameters.joints[current_joint]["axis"] = strToVec3(attributes_dict["xyz"])
				"limit":
					RobotParameters.joints[current_joint]["limit"]["lower"] = float(attributes_dict["lower"])
					RobotParameters.joints[current_joint]["limit"]["upper"] = float(attributes_dict["upper"])
					RobotParameters.joints[current_joint]["limit"]["effort"] = float(attributes_dict["effort"])
					RobotParameters.joints[current_joint]["limit"]["velocity"] = float(attributes_dict["velocity"])
				"link":
					current_link = attributes_dict["name"]
					RobotParameters.links[current_link] = link_dict.duplicate(true)
					is_joint = false
				"inertial":
					current_attribute = "inertial"
				"visual":
					current_attribute = "visual"
				"collision":
					current_attribute = "collision"
				"geometry":
					current_subattribute = "geometry"
				"origin":
					if is_joint:
						RobotParameters.joints[current_joint]
						RobotParameters.joints[current_joint]["origin"]["xyz"] = strToVec3(attributes_dict["xyz"])
						RobotParameters.joints[current_joint]["origin"]["rpy"] = strToVec3(attributes_dict["rpy"])
					else:
						RobotParameters.links[current_link][current_attribute]["origin"]["xyz"] \
							= strToVec3(attributes_dict["xyz"])
						RobotParameters.links[current_link][current_attribute]["origin"]["rpy"] \
							= strToVec3(attributes_dict["rpy"])
				"mass":
					RobotParameters.links[current_link]["inertial"]["mass"] = float(attributes_dict["value"])
				"inertia":
					RobotParameters.links[current_link]["inertial"]["inertia"]["ixx"] = float(attributes_dict["ixx"])
					RobotParameters.links[current_link]["inertial"]["inertia"]["ixy"] = float(attributes_dict["ixy"])
					RobotParameters.links[current_link]["inertial"]["inertia"]["ixz"] = float(attributes_dict["ixz"])
					RobotParameters.links[current_link]["inertial"]["inertia"]["iyy"] = float(attributes_dict["iyy"])
					RobotParameters.links[current_link]["inertial"]["inertia"]["iyz"] = float(attributes_dict["iyz"])
					RobotParameters.links[current_link]["inertial"]["inertia"]["izz"] = float(attributes_dict["izz"])
				"mesh":
					RobotParameters.links[current_link][current_attribute][current_subattribute]["mesh"] \
						= attributes_dict["filename"]
					var mesh_filepath = RobotParameters.urdf_dir + attributes_dict["filename"]
					var mtl_filepath = ""
					if (FileAccess.file_exists(mesh_filepath.trim_suffix(".obj") + ".mtl")):
						mtl_filepath = mesh_filepath.trim_suffix(".obj") + ".mtl"
					var obj:Mesh = ObjParse.load_obj(mesh_filepath, mtl_filepath)
					RobotParameters.links[current_link][current_attribute][current_subattribute]["obj"] \
						= obj
				"material":
					RobotParameters.links[current_link][current_attribute][current_subattribute]["material"] \
						= attributes_dict["name"]
				"color":
					RobotParameters.links[current_link][current_attribute][current_subattribute]["color"] \
						= strToVec4(attributes_dict["rgba"])
				"box":
					RobotParameters.links[current_link][current_attribute][current_subattribute]["box"] \
						= {}
					RobotParameters.links[current_link][current_attribute][current_subattribute]["box"]["size"] \
						= strToVec3(attributes_dict["size"])
				"cylinder":
					RobotParameters.links[current_link][current_attribute][current_subattribute]["cylinder"] \
						= {}
					RobotParameters.links[current_link][current_attribute][current_subattribute]["cylinder"]["radius"] \
						= float(attributes_dict["radius"])
					RobotParameters.links[current_link][current_attribute][current_subattribute]["cylinder"]["length"] \
						= float(attributes_dict["length"])
				"sphere":
					RobotParameters.links[current_link][current_attribute][current_subattribute]["sphere"] \
						= {}
					RobotParameters.links[current_link][current_attribute][current_subattribute]["sphere"]["radius"] \
						= float(attributes_dict["radius"])
	Signals.emit_signal("RobotLoaded")

static func strToVec3(string := "") -> Vector3:
	if string:
		var new_string: String = string
		var array: Array = new_string.split(" ")
		return Vector3(float(array[0]), float(array[1]), float(array[2]))
	return Vector3.ZERO
	
static func strToVec4(string := "") -> Vector4:
	if string:
		var new_string: String = string
		var array: Array = new_string.split(" ")
		return Vector4(float(array[0]), float(array[1]), float(array[2]), float(array[3]))
	return Vector4.ZERO
