# This script does not work at all after the ssh :(
echo "Connecting..."
$PORT = 5800
$jarpath = $PSScriptRoot + "\photonvision-v2022.1.6.jar"
scp -P $PORT $jarpath pi@gloworm.local
ssh pi@gloworm.local -p $PORT
sudo systemctl stop photonvision.service
sudo mv $jarpath /opt/photonvision/photonvision.jar
sudo systemctl start photonvision.service
echo "...Done"