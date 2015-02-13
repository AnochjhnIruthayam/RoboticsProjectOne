 This example illustrates how to use cmake to create a plugin application
 for RobWorkStudio that uses OpenCV.
 
 When compiled, the output will be a dynamically linkable file (Windows: .dll - Linux: .so). 
 To use the plugin in your robworkstudio installation add the following line to the 
 RobWorkStudio.ini file in the RobWorkStudio.exe directory (Windows style):
 
	SamplePlugin\DockArea=2
	SamplePlugin\Filename=libSamplePlugin
	SamplePlugin\Path=c:/workspace/RobVis1Plugin2013/libs/release
	SamplePlugin\Visible=true

 Add the following line to the RobWorkStudio.ini file in the RobWorkStudio.exe directory (Linux style):
 
	SamplePlugin\DockArea=2
	SamplePlugin\Filename=libSamplePlugin
	SamplePlugin\Path=/workspace/RobVis1Plugin2013/libs/release
	SamplePlugin\Visible=true