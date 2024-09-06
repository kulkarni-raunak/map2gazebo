# ROS package for creating Gazebo environments from 2D maps

Subscribes to your map topic and exports a mesh for use in Gazebo in the
destination folder you specify.  The mesh will have obstacles (tall boxes)
corresponding to all occupied map squares.  Requires a map to be publishing
somewhere, and will probably work better if the map is static; I recommend
using `map_server` with a saved map. 
Or if you have nav2 then use `nav2_map_server`.

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={path_to_map_yaml} 
```

If run with default parameters, it will write the mesh to this package's
models/map/meshes folder.  TODO: You will then be able to run `gazebo_world.launch`
to launch Gazebo pre-populated with the map mesh.  

## Arguments and parameters

The export directory is specified as a launchfile argument.  Change it using

```bash
ros2 launch map2gazebo map2gazebo.launch.py export_dir:=/path/to/export_dir
```

Note that if you change the export directory, `gazebo_world.launch` will not
work unmodified.

Default parameters are specified in config/defaults.yaml; the location of the
YAML parameter file is also a launchfile argument.  To change the defaults,
make a new YAML parameter file, and run

```bash
ros2 launch map2gazebo map2gazebo.launch.py params_file:=/path/to/your/params.yaml
```

Alternatively you could just edit the default parameter file (not recommended,
but I can't stop you).

YAML parameters:

* `map_topic`: Topic that the map to convert is being published on.
* `mesh_type`: Can be "stl" or "dae"; sets whether to export the mesh as a stl (default) or as a Collada file.  Note that dae files specify a color and that by default this color is black, which you will likely want to edit as it makes your world very hard to see.  If exporting as dae, you will also need to modify `models/map/model.sdf` to specify `map.dae` instead of `map.stl` if you are planning to use `gazebo_world.launch`.
* `occupied_thresh`: Minimum threshold value for considering a cell occupied. Defaults to 1 (out of 100).  
* `box_height`: Height of boxes in gazebo environment.  Defaults to 2m.

The YAML file does not specify the export directory because it doesn't seem to
support a value with substitution args like
`"$(find map2gazebo)/models/map/meshes"`, which is the desired default value.
The roslaunch `<arg>` and `<param>` tags are therefore used, but unfortunately
the `<param>` tag overrides any non-default value loaded from the YAML file.
Therefore, don't use the YAML parameter file to set a non-default export
directory.  (Let me know, or better submit a PR, if you know of a more elegant
way to do this!)

## Installation

Clone the repo and install ROS dependencies with rosdep.  

Install the python dependencies with pip:

```bash
pip install --user trimesh
pip install --user numpy
```

trimesh needs the following soft dependencies to export Collada (.dae) files.
Theoretically you can install these with `pip install trimesh[soft]` but this
failed for me, so I installed the needed ones myself.

```bash
pip install --user pycollada
pip install --user scipy
pip install --user networkx
```

## Suggested after work

* Open the mesh in software called MesbLab and apply filter called "Surface Reconstruction: Screened Poisson" under "Remeshing, Simplification and Reconstruction" and save the file. This reduces the size to great extent so that it can be easily rendered in Gazebo (test on Gazebo Garden)
* One can also apply texture to the mesh using MeshLab, go to filters -> Texture -> Set Texture -> Add the png or jpg file.
* Add model to Gazebo's resource path.

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$WORKSPACE/install/share/map2gazebo/models/
```

* Open empty world. `gz sim empty.sdf` and add your model and other models if required.
* Save the world to .sdf file using

```bash
gz service -s /world/empty/generate_world_sdf --reqtype gz.msgs.SdfGeneratorConfig --req 'global_entity_gen_config: { expand_include_tags: { data: true } }' --timeout 1000 --reptype gz.msgs.StringMsg | \
sed -e 's/^data: "//' -e 's/"$//' -e 's/\\n/\n/g' -e "s/\\\'/'/g" > custom_world.sdf
```

## Kml to PGM converter

Edit the file name in the program "map2gazebo/kml_to_pgm_yaml.py" with relative paths. This will generate a PGM and YAML file from the provided kml file containing polygon data. Note: A padded layer in added to the generated grid.
