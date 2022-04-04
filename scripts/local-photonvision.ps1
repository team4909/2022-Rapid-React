$jarpath = $PSScriptRoot + "\photonvision-v2022.1.6.jar"
java -jar $jarpath --disable-networking
start "" localhost:5800