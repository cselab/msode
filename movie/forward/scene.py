# state file generated using paraview version 5.8.1

# ----------------------------------------------------------------
# setup views used in the visualization
# ----------------------------------------------------------------

# trace generated using paraview version 5.8.1
#
# To ensure correct image size when batch processing, please search
# for and uncomment the line `# renderView*.ViewSize = [*,*]`

#### import the simple module from the paraview
from paraview.simple import *
#### disable automatic camera reset on 'Show'
paraview.simple._DisableFirstRenderCameraReset()

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('wc', type=int)
parser.add_argument('step', type=int)
args = parser.parse_args()

# Create a new 'Light'
light1 = CreateLight()
light1.Intensity = 0.5
light1.Position = [-1.049520986211406, 0.38571569075704826, -3.4188131159918353]
light1.FocalPoint = [-1.049520986211406, 0.38571569075704826, 0.24393297521607865]

# a texture

# get the material library
materialLibrary1 = GetMaterialLibrary()

# create light
# Create a new 'Render View'
renderView1 = CreateView('RenderView')
renderView1.ViewSize = [2003, 1172]
renderView1.AxesGrid = 'GridAxes3DActor'
renderView1.OrientationAxesVisibility = 0
renderView1.CenterOfRotation = [-1.4734458476305008, 0.15178757905960083, -1.4532357454299927e-05]
renderView1.UseLight = 0
renderView1.StereoType = 'Crystal Eyes'
renderView1.CameraPosition = [-1.049520986211406, 0.38571569075704826, -3.4188131159918353]
renderView1.CameraFocalPoint = [-1.049520986211406, 0.38571569075704826, 0.24393297521607865]
renderView1.CameraFocalDisk = 1.0
renderView1.CameraParallelScale = 1.1470660193931008
renderView1.Background = [1.0, 1.0, 1.0]
renderView1.EnableRayTracing = 1
renderView1.BackEnd = 'OSPRay pathtracer'
renderView1.SamplesPerPixel = 8
renderView1.AdditionalLights = light1
renderView1.OSPRayMaterialLibrary = materialLibrary1

res = renderView1.ViewSize

SetActiveView(None)

# ----------------------------------------------------------------
# setup view layouts
# ----------------------------------------------------------------

# create new layout object 'Layout #1'
layout1 = CreateLayout(name='Layout #1')
layout1.AssignView(0, renderView1)

# ----------------------------------------------------------------
# restore active view
SetActiveView(renderView1)
# ----------------------------------------------------------------

# ----------------------------------------------------------------
# setup the data processing pipelines
# ----------------------------------------------------------------

# create a new 'PLY Reader'
pLYReader1 = PLYReader(FileNames=['/Users/amoudruz/projects/msode/movie/forward/ply_swimmer_00_wc_{:d}/swimmer_00_{:05d}.ply'.format(args.wc, args.step)])

# create a new 'Plane'
plane1 = Plane()
plane1.Origin = [-4.0, -4.0, 1.0]
plane1.Point1 = [4.0, -4.0, 1.0]
plane1.Point2 = [-4.0, 4.0, 1.0]

# create a new 'PLY Reader'
pLYReader2 = PLYReader(FileNames=['/Users/amoudruz/projects/msode/movie/forward/ply_swimmer_01_wc_{:d}/swimmer_00_{:05d}.ply'.format(args.wc, args.step)])

# create a new 'Transform'
transform1 = Transform(Input=pLYReader2)
transform1.Transform = 'Transform'

# init the 'Transform' selected for 'Transform'
transform1.Transform.Translate = [0.0, 0.8, 0.0]

# create a new 'Loop Subdivision'
loopSubdivision2 = LoopSubdivision(Input=transform1)
loopSubdivision2.NumberofSubdivisions = 4

# create a new 'Loop Subdivision'
loopSubdivision1 = LoopSubdivision(Input=pLYReader1)
loopSubdivision1.NumberofSubdivisions = 4

# ----------------------------------------------------------------
# setup the visualization in view 'renderView1'
# ----------------------------------------------------------------

# show data from plane1
plane1Display = Show(plane1, renderView1, 'GeometryRepresentation')

# trace defaults for the display properties.
plane1Display.Representation = 'Surface'
plane1Display.ColorArrayName = ['POINTS', '']
plane1Display.Texture = None
plane1Display.OSPRayScaleArray = 'Normals'
plane1Display.OSPRayScaleFunction = 'PiecewiseFunction'
plane1Display.OSPRayMaterial = 'tiles'
plane1Display.SelectOrientationVectors = 'None'
plane1Display.ScaleFactor = 7.0
plane1Display.SelectScaleArray = 'None'
plane1Display.GlyphType = 'Arrow'
plane1Display.GlyphTableIndexArray = 'None'
plane1Display.GaussianRadius = 0.35000000000000003
plane1Display.SetScaleArray = ['POINTS', 'Normals']
plane1Display.ScaleTransferFunction = 'PiecewiseFunction'
plane1Display.OpacityArray = ['POINTS', 'Normals']
plane1Display.OpacityTransferFunction = 'PiecewiseFunction'
plane1Display.DataAxesGrid = 'GridAxesRepresentation'
plane1Display.PolarAxes = 'PolarAxesRepresentation'

# init the 'PiecewiseFunction' selected for 'ScaleTransferFunction'
plane1Display.ScaleTransferFunction.Points = [0.0, 0.0, 0.5, 0.0, 1.1757813367477812e-38, 1.0, 0.5, 0.0]

# init the 'PiecewiseFunction' selected for 'OpacityTransferFunction'
plane1Display.OpacityTransferFunction.Points = [0.0, 0.0, 0.5, 0.0, 1.1757813367477812e-38, 1.0, 0.5, 0.0]

# show data from loopSubdivision1
loopSubdivision1Display = Show(loopSubdivision1, renderView1, 'GeometryRepresentation')

# trace defaults for the display properties.
loopSubdivision1Display.Representation = 'Surface'
loopSubdivision1Display.ColorArrayName = ['POINTS', '']
loopSubdivision1Display.OSPRayScaleFunction = 'PiecewiseFunction'
loopSubdivision1Display.OSPRayMaterial = 'scratched'
loopSubdivision1Display.SelectOrientationVectors = 'None'
loopSubdivision1Display.ScaleFactor = 0.09974263906478882
loopSubdivision1Display.SelectScaleArray = 'None'
loopSubdivision1Display.GlyphType = 'Arrow'
loopSubdivision1Display.GlyphTableIndexArray = 'None'
loopSubdivision1Display.GaussianRadius = 0.004987131953239441
loopSubdivision1Display.SetScaleArray = ['POINTS', '']
loopSubdivision1Display.ScaleTransferFunction = 'PiecewiseFunction'
loopSubdivision1Display.OpacityArray = ['POINTS', '']
loopSubdivision1Display.OpacityTransferFunction = 'PiecewiseFunction'
loopSubdivision1Display.DataAxesGrid = 'GridAxesRepresentation'
loopSubdivision1Display.PolarAxes = 'PolarAxesRepresentation'

# show data from loopSubdivision2
loopSubdivision2Display = Show(loopSubdivision2, renderView1, 'GeometryRepresentation')

# trace defaults for the display properties.
loopSubdivision2Display.Representation = 'Surface'
loopSubdivision2Display.ColorArrayName = ['POINTS', '']
loopSubdivision2Display.OSPRayScaleFunction = 'PiecewiseFunction'
loopSubdivision2Display.OSPRayMaterial = 'scratched'
loopSubdivision2Display.SelectOrientationVectors = 'None'
loopSubdivision2Display.ScaleFactor = 0.09912500083446503
loopSubdivision2Display.SelectScaleArray = 'None'
loopSubdivision2Display.GlyphType = 'Arrow'
loopSubdivision2Display.GlyphTableIndexArray = 'None'
loopSubdivision2Display.GaussianRadius = 0.004956250041723251
loopSubdivision2Display.SetScaleArray = ['POINTS', '']
loopSubdivision2Display.ScaleTransferFunction = 'PiecewiseFunction'
loopSubdivision2Display.OpacityArray = ['POINTS', '']
loopSubdivision2Display.OpacityTransferFunction = 'PiecewiseFunction'
loopSubdivision2Display.DataAxesGrid = 'GridAxesRepresentation'
loopSubdivision2Display.PolarAxes = 'PolarAxesRepresentation'

# ----------------------------------------------------------------
# finally, restore active source
SetActiveSource(plane1)
# ----------------------------------------------------------------

SaveScreenshot("wc_{:d}_{:05d}.png".format(args.wc, args.step), layout1, SaveAllViews=1, ImageResolution=res)
