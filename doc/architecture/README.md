# Droid Control Ship <!-- omit in toc -->

- [SW Architecture](#sw-architecture)
- [Abbreviations](#abbreviations)
  - [Physical View](#physical-view)
  - [Development View](#development-view)
    - [Layers](#layers)
  - [Logical View](#logical-view)
    - [Application](#application)
    - [HAL](#hal)
- [Process View](#process-view)
- [Issues, Ideas And Bugs](#issues-ideas-and-bugs)
- [License](#license)
- [Contribution](#contribution)

## SW Architecture

The Droid Control Ship repository contains several applications. Each application is described according to the 4+1 architectural view.

![architecturalView](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/architecture/uml/ViewModels.plantuml)

## Abbreviations

| Abbreviation | Description |
| - | - |
| HAL | Hardware Abstraction Layer |

### Physical View

The physical view shows the deployment which is equal to all applications. But not every application may use all of the provided sensors and actors.

TODO

### Development View

#### Layers

TODO

### Logical View

#### Application

The following applications are supported:

TODO

#### HAL

The hardware abstraction layer (HAL) for the target is shown. For the simulation and the test only the realization part is different and may base on other 3rd party components.

![hal](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/BlueAndi/DroidControlShip/main/doc/architecture/uml/LogicalView/HAL.plantuml)

## Process View

TODO

## Issues, Ideas And Bugs

If you have further ideas or you found some bugs, great! Create a [issue](https://github.com/BlueAndi/DroidControlShip/issues) or if you are able and willing to fix it by yourself, clone the repository and create a pull request.

## License

The whole source code is published under the [MIT license](http://choosealicense.com/licenses/mit/).
Consider the different licenses of the used third party libraries too!

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, shall be licensed as above, without any
additional terms or conditions.
