# This script does not work at all after the ssh :(
echo "Connecting..."
$limelightIP = "10.49.9.42"
$jarpath = $PSScriptRoot + "\photonvision-v2022.1.6.jar"
scp -P 5800 $jarpath pi@gloworm.local
echo "SSHing"
ssh pi@gloworm.local
sudo systemctl stop photonvision.service
sudo mv $jarpath /opt/photonvision/photonvision.jar
sudo systemctl start photonvision.service
echo "...Done"