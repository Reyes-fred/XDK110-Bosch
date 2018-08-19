mkdir xdkhome
cd xdkhome

filepath=$PWD

docker run -d --name="hass" --restart on-failure -v $filepath:/config -e "TZ=America/Chicago" -p 8123:8123 homeassistant/home-assistant
