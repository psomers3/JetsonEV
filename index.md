---
layout: default
---
<!-- Include the following line in any markdown file to use LaTeX code. -->
<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script> 

# JetsonEV
This is a package meant for controlling the electric vehicles at the Institute for System Dynamics. These vehicles are based on an RC car platform and driven by either a Jetson Nano or Raspberry Pi (not yet tested). This package is intended to provide easy, high-level access and control of the on-board hardware.

## Installation
```console
user@JetsonEV:~$ git clone https://github.psomers3/JetsonEV.git
user@JetsonEV:~$ cd JetsonEV
user@JetsonEV:~$ sudo pip install -r requirements.txt
user@JetsonEV:~$ sudo pip install .
```

## Code Documentation
[Link to JetsonEV Documentation](./JetsonEV.html)

A detailed description of each mode for the JetsonEV can be found [here](./modes.html).

See [examples](https://github.tik.uni-stuttgart.de/ac121730/JetsonEV/tree/master/examples) for examples how to use JetsonEV.

The documentation for the JetsonEV class was generated using the latest development version of [pydoc-markdown](https://pypi.org/project/pydoc-markdown/). It was done while on the gh-pages branch with the command:

```bash
pydoc-markdown -m JetsonEV.JetsonEV --render-toc > JetsonEV.md
```

## Hardware Documentation
[Link to hardware information](./Hardware.html)

## Startup GUI
The JetsonEV is configured to start up with just a bare bones GUI made for a small touchscreen. This GUI is located [here](https://github.com/psomers3/TouchUI) and is a fork of a separate project. Click [here](./GUI.html) for a brief overview of how to use the GUI.
