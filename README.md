# Wall-E

## Folder Structure
- Example: contains helper and sample scripts
- Pictures: contains some sample pictures for testing as well as sample output images
- drive: contains all scripts necessary for Wall-E to drive as well as the Tensorflow model

## Getting started
1. Connect Wall-E/ the pi-top to WiFi
Wall-E is configured to connect to the Nao router in the HdM AI Lab.
If you want to change the network settings perform these steps:
  * Connect to the pi-top following these [instructions](https://knowledgebase.pi-top.com/knowledge/wifi-access-point "Connect to your pi-top") ([additional link](https://www.pi-top.com/start/ap-connection "Change network settings"))
  * Then change the network settings directly on the website that should pop up after opening up pi-top.local (or 192.168.64.1) in your browser.
2. Access the pi-top Desktop by following either [steps](https://www.pi-top.com/start/getting-started-page23?hsCtaTracking=4a97a9b7-7ac7-48a5-a6dd-52b923a5e7c7%7C10d7fe60-a434-4cc4-b212-fe1340b101f3 "Open up pi-top Desktop").
3. Run Visual Studio Code and open the repository which is located under "/home/pi/Documents/repo/Wall-E".
4. Run rover.py which can be found in the Example folder by opening the script and clicking on the green arrow in the upper corner on the right hand side.
5. Wall-E should now start to drive according to the script if everything was set up correctly.
6. If you place Wall-E on the fake road (can be found in the HdM AI Lab) you can then run car.py the same way you've already run rover.py and watch it perform the tasks described in the documentation which is located in the same folder as this README.

Have fun and don't forget that the strip club is always an alternative :))
