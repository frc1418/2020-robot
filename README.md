# 2019 Robot Code
**Robot Code** | [Dashboard](https://github.com/frc1418/2020-dashboard) | [Vision](https://github.com/frc1418/2020-vision)

[![Build Status](https://travis-ci.com/frc1418/2019-robot.svg?token=VDF6qgkKLeZhHqMRYJnC&branch=master)](https://travis-ci.com/frc1418/2020-robot)

> Code for Team 1418's 2019 competition robot, which is so far unnamed.

## Deploying onto the robot
Before deploying, you must [install robotpy](http://robotpy.readthedocs.io/en/stable/install/robot.html#install-robotpy) on your robot.

You may then deploy code at any time:

	python3 src/robot.py deploy

During development of last year's robot code, we created a Bash script `deploy.sh` to automate some tasks related to code deploy. You can find that tool, `dep`, [here](https://github.com/frc1418/dep). We recommend that you make use of it to simplify your deploy process and remove pesky steps like manually changing your WiFi network.

## Testing/Simulation
You may run the `pyfrc` simulator to test this code thus:

    python3 src/robot.py sim

## Controls
We use three total joysticks to control the robot:

* 2 x **Logitech Attack 3** (`joystick_left` and `joystick_right`)
* 1 x **Logitech Extreme 3D Pro** (`joystick_alt`)

<img src="res/ATK3.png" height="600"><img src="res/X3D.png" height="600">

# Basic Setup

## Setting up `git` hooks:

`git` hooks change the process of committing by adding processes before or after the process of committing. After cloning, you should run

	./setup.sh

This will set up hooks to run tests before committing to help avoid easy-to-fix errors in the code.

## Installing Dependencies

The `requirements.txt` file holds all of the programs and their versions needed to run this code. After cloning, run

	pip3 install -r requirements.txt

Add `--user` at the end of that command if you do not have admin privileges.

In the `tests` folder there is another requirements file that needs to be run. To do so, run

	pip3 install -r tests/requirements.txt

Add `--user` at the end of that command if you do not have admin privileges.

## Setting up entry points

Entry points allow us to add sub-commands to the overarching RobotPY framework. To load them, move to the root directory, activate your virtual environment and run

	pip install .

This will set up the entry points that we put in place to facilitate off-robot functions of our code. So far, we use entry points for:
* Generating motion profile trajectories (generate)

These entry points can be run like so

    python3 src/robot.py <entry_point_name>

## Changing your `$PATH`

Your `$PATH` is a variable that contains a bunch of different directories that are searched through when the computer is searching for an executable file. To run this robot code, `$HOME/Library/Python/3.6/bin` needs to be added to your `$PATH`. To do so, first run

	cd ~/

This will bring you to your home directory. If you want to open up your `.bash_profile` (where the `$PATH` is stored, NOTE: The name of this file will likely change depending one's platform, chosen shell, terminal settings, etc.) using nano, run

	nano .bash_profile

Once you have done one of these, add the following line of code to `.bash_profile.`

  	PATH=$PATH:$HOME/Library/Python/3.6/bin

This will save your changes. If you used nano, save your changes and exit with

	ctrl + x

Next, activate your changes in nano with

	source .bash_profile

To see if the changes took place, run

	echo $PATH

When this is run, you should see the addition of `Library/Python/3.6/bin` at the end of your `$PATH`.

## File Structure

    src/
    	The robot code lives here.
        automations/
            Automatic scripts for performing common functions.
        autonomous/
            Autonomous modes.
        common/
            New robotpy components.
        components/
            Abstractions for major robot systems.
        controllers/
            Software implementations not corresponding to physical robot components.
	tests/
		py.test-based unit tests that test the code and can be run via pyfrc.

## Authors
* Team 1418

## Licensing
In-season, use of this software is restricted by the FRC rules. After the season ends, the [MIT License](LICENSE) applies instead.
