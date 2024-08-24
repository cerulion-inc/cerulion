extends Control

@onready var window_text:Label = $VBoxContainer/Panel/Label
var filepath:String = ""
var xml_parser:XMLParser = XMLParser.new()
var USER_DIR:DirAccess = DirAccess.open("user://")
var urdf_name:String = ""
var link_dict:Dictionary = {
	
}

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	get_viewport().files_dropped.connect(onFilesDropped)
	if (not USER_DIR.dir_exists("robots")):
		USER_DIR.make_dir("robots")

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	pass

func onFilesDropped(files):
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
			if (not f.begins_with("_")):
				print(f)
				if (f.ends_with("/")):
					USER_DIR.make_dir_recursive("robots/" + f)
				else:
					if (f.ends_with(".urdf")):
						urdf_name = (f.get_file()).trim_suffix(".urdf")
					var write_file = FileAccess.open("user://robots/" + f, FileAccess.WRITE)
					var file_contents = zip_reader.read_file(f)
					write_file.store_buffer(file_contents)
	parseURDF("user://robots/" + urdf_name + ".urdf")

func parseURDF(urdf_file):
	var parser = XMLParser.new()
	parser.open(urdf_file)
	var current_link:String = ""
	var current_joint:String = ""
	var urdf_dict = {}
	var current_element_stack = []
	var current_dict = urdf_dict
	
	while parser.read() != ERR_FILE_EOF:
		if parser.get_node_type() == XMLParser.NODE_ELEMENT:
			pass
			#print("The ", node_name, " element has the following attributes: ", attributes_dict)
	print(urdf_dict)
