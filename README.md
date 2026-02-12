# ink3d
Simple cpp application to render an input 3d file (.off, .obj, .stl etc) as a SVG drawing in the style of technical ink drawing.

<img src="samples/eg_result.svg" width="200" alt="Example output"/>

## Requirements
- Linux
- [conan >=2.25.2](https://conan.io/downloads)

## Dependencies
- CGAL 6.1

## Build
- After installing conan, run `bash build.sh`. This will install all the requirements and build the program.
- A binary `ink3d` will appear in the `build` folder.
- If you wish to create a statically linked bin, uncomment the lines according to the comments in `conanfile.txt`

## Usage
```bash
ink3d path/to/config.json
# e.g.,
# ink3d samples/config.json
```
After running the above command, an svg file will appear according to the
configuration file supplied. See samples/config.json for an example.

```json
// config.json
{
	"input_path": "./samples/cubes.stl",
	"output_path": "./output.svg",
	"view_origin": [1,1,1],
	"look_at": [0,0,0],
	"projection": "perspective", // perspective or orthographic
	"samples": 108,
	"refine_steps": 108
}
```