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
ink3d <3d model file path>
# e.g.,
# ink3d samples/cubes.stl
```
After running the above command, an `output.svg` will appear in the same folder

## Limitations
- View angle is currently fixed and only views from points `[1,1,1]` to `[0,0,0]`.
