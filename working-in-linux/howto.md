# Working with OpenWSN on Linux Ubuntu 14.04

| *document* | Working with OpenWSN on Linux Ubuntu 14.04                   |
|:----------:|:-------------------------------------------------------------|
| *version*  |  1.0                                                         |
| *date*     |  July, 7th 2015                                              |
| *author*   |  Guillaume Gaillard, <guillaume.gaillard@orange.com>         |


## Table of Contents

|               Sections                                                 |
|:----------------------------------------------------------------------:|
| [Foreword](#foreword)                                                  |
| [Clone the repo]#(clone-the-repo)					 |
| [Syncing with berkeley]#(syncing with berkeley)			 |
| [Compile the code]#(compile the code)					 |
| [Push to the forge]#(push to the forge)				 |

## Foreword
Some guidelines on the flow to use OpenWSN Firmware on ubuntu.
*Note*: This tutorial is actually notes I've taken when syncing the Orange forge repo with the berkeley repo.
*Note*: More tutorials on github syncing a fork.
*Note*: Not intended to replace OpenWSN Kickstart-Linux tutorial.
*Note*: Not intended to replace future @Jonathan @Denglin tutorials about Running OpenWSN on M3 nodes. 

## Clone the repo
You may have done that already if you read this.
If you are out of office you won't have the certificate

```
env GIT_SSL_NO_VERIFY=true git clone https://www.forge.orange-labs.fr/plugins/git/slab2/node.git node
cd node
```

## Syncing with berkeley
Pre-requisite : you need to fork the github repo berkeley/openwsn-fw. I use my fork Guillaumegaillard

Clone your fork in a temp folder :
```
node$ git clone https://github.com/Guillaumegaillard/openwsn-fw.git temp
node$ cd temp/
```

Copy your fork configuration into Orange:
```
node/temp$ cp -r .git ../openwsn-fw-orange/
node/temp$ cd ../openwsn-fw-orange/
node/openwsn-fw-orange$ rm -rf ../temp/
```

Add remote reference to the main openwsn repo
```
node/openwsn-fw-orange$ git remote add upstream https://github.com/openwsn-berkeley/openwsn-fw.git
node/openwsn-fw-orange$ git remote -v
	origin	https://github.com/Guillaumegaillard/openwsn-fw.git (fetch)
	origin	https://github.com/Guillaumegaillard/openwsn-fw.git (push)
	upstream	https://github.com/openwsn-berkeley/openwsn-fw.git (fetch)
	upstream	https://github.com/openwsn-berkeley/openwsn-fw.git (push)
```

Configure your git :
```
node/openwsn-fw-orange$ git config --global user.email "guillaume.gaillard.maze@gmail.com"
node/openwsn-fw-orange$ git config --global user.name "Guillaume Gaillard"
```

Locally commit the repo in the Orange state, to prepare the syncing. 
```
node/openwsn-fw-orange$ git commit -a -m "prepare syncing Orange FW with Berk fw"
```
DO NOT PUSH HERE TO YOUR FORK IF THE PROJECT IS NOT PUBLIC.
```
node/openwsn-fw-orange$ #\!/#git push#/!\##
```

Download berkeley's current code: 
```
node/openwsn-fw-orange$ git fetch upstream 
```

Merge the repo with current code. Edit and correct the files where conflicts appear.
```
node/openwsn-fw-orange$ git merge upstream/develop
	Auto-merging projects/telosb/SConscript.env
	...
	CONFLICT (content): Merge conflict in openstack/cross-layers/openqueue.c
	CONFLICT (content): Merge conflict in openstack/03b-IPv6/forwarding.c
	...
	Automatic merge failed; fix conflicts and then commit the result.

gedit openstack/cross-layers/openqueue.c openstack/03b-IPv6/forwarding.c ...
```

## Compile the code
Install OpenWSN Kickstart tools
```
:~$ sudo apt-get install -y python-dev scons python-pip python-tk
:~$ sudo pip install bottle
:~$ sudo pip install PyDispatche
```

Clone github exp-iotlab repo in order to obtain the gcc-arm compiler (in order to compile the M3 bin)
TODO:
```
git clone ...
```
or as Quentin pointed out :
```
grab the arm-gcc compiler for ARM-cortex-M processors from [here](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update/+download/gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2) or on their main [page](https://launchpad.net/gcc-arm-embedded) for newer versions.
```

:~$ cd path/to/.../exp-iotlab/
exp-iotlab$ make ensure-gcc-arm 
exp-iotlab$ export PATH=$PATH:/home/.../path/to/.../exp-iotlab/local/gcc-arm-none/bin
:~$ cd path/back/to/.../node/openwsn-fw-orange
```

Edit the make file, create the bin folder if not present and build
```
node/openwsn-fw-orange$ (mkdir bin)
node/openwsn-fw-orange$ ./make
```

## Push to the forge
Add relevant files to the commit (merged files, new files, etc.) and commit to your fork :
```
node/openwsn-fw-orange$ git status
node/openwsn-fw-orange$ git add ...
node/openwsn-fw-orange$ node$ git commit -m "merge with sensorlab 07-07-2015"
```
DO NOT PUSH HERE TO YOUR FORK IF THE PROJECT IS NOT PUBLIC.
```
node/openwsn-fw-orange$ #\!/#git push#/!\##
```

Add relevant files to the commit (merged files, new files, etc.) and commit to your fork :
```
node/openwsn-fw-orange$ cd ..
node$ git status
node$ git add ...
node$ node$ git commit -m "merge with sensorlab 07-07-2015"
```
DO NOT PUSH HERE TO THE FORGE IF UNSURE OF THE RESULT.
```
node$ #\!/#git push#/!\##
```



