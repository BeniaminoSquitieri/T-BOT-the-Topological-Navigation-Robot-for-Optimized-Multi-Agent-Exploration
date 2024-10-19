#Test internet connection
wget www.google.it -O /tmp/test.html
if [[ $? != 0 ]]; then
    echo "Internet connection not available"
    exit 1
fi

if [ ! -d "/tmp/turtlebot4_setup_orig/" ]; then
	git clone -b humble https://github.com/turtlebot/turtlebot4_setup.git /tmp/turtlebot4_setup_orig/
	echo "Return: $?" 
fi
