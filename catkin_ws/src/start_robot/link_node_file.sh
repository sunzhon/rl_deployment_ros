#! /bin/sh

# create link to make the build node file (devel/lib/bin) to be at (src/stbot/), thereby can be found by launch file


creat_link(){
    if [ $# -lt 2 ]; then
        echo "argument num: $#, creat link function has less than two arguments"
        exit
    else
        echo "start to creat link files....."
    fi
    source_file=$1
    target_file=$2

    echo "source file: $source_file"
    echo "target file: $target_file"

    if [ -f "$source_file" ]; then
        if [ -L "$target_file" ]; then
            echo "remove exist link and create a new link"
            rm $target_file
        else
            echo "creat a link"
        fi
        ln -s $source_file $target_file && echo  "creat node link successfully!"
    else
        echo "source file does not exist"
        exit
    fi
}
echo "start link node"
source_file="$AMBOT/projects/catkin_ws/devel/lib/ambot_controller/ambot_controller_node"
target_file="$AMBOT/projects/catkin_ws/src/ambot_controller/ambot_controller_node"
creat_link $source_file $target_file

source_file="$AMBOT/projects/catkin_ws/devel/lib/ambot_realrobot/ambot_realrobot_node"
target_file="$AMBOT/projects/catkin_ws/src/ambot_realrobot/ambot_realrobot_node"
creat_link $source_file $target_file

