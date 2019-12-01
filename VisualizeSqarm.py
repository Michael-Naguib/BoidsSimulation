'''
# Visualize Swarm
- Code By Michael Sherif Naguib
- license: MIT
- Date: 11/25/19
- @University of Tulsa
- Description: Code to visualize a swarm using a library used to visualize point clouds....
'''

# Imports
from open3d import open3d
import time

class VisualizeSwarm():
    def __init__(self,frameDelay=0.0,box=None,pointSize=5,addSphereAtOrigin=True):
        '''
        :description:       init the visualization code for vectors in 3d space with color
        :param frameDelay:  an extra delay to add between frames
        :param box:         if specified creates a 3d box with diagonals in the view
        '''

        # Init Settings
        self.firstFrame = True
        self.box = box
        self.frameDelay = frameDelay
        self.pointSize = pointSize

        # Init the viewer
        self.pcd = open3d.geometry.PointCloud()
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(self.pcd)

        # Add the box if specified
        if bool(self.box):
            mesh_box = open3d.geometry.TriangleMesh.create_box(width=box[0], height=box[1], depth=box[1])
            wireBox = open3d.geometry.LineSet.create_from_triangle_mesh(mesh_box)
            self.vis.add_geometry(wireBox)
        if addSphereAtOrigin:
            mesh_sphere = open3d.geometry.TriangleMesh.create_sphere(radius=20.0)
            mesh_sphere.compute_vertex_normals()
            mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
            self.vis.add_geometry(mesh_sphere)

        #TODO: WARNING THIS LINE MAY CAUSE AN ERROR --> here I excluded setting the first frame as I did in view_sim.py
        #      and do not know why I did then ... if it errors in a weird way ... check this first....
        # Specify the point size
        render_option = self.vis.get_render_option()
        render_option.point_size = self.pointSize
    def tick(self,frameData,converted=False):
        '''
        :description: Visualize one frame of the data
        :param converted:  The arrays must be converted to a special open3d type before they can be displayed
                           if the frames are already converted then do not do a redundant conversion... can be used for
                           efficiency purposes...
        :param frameData:
                    [
                        positionData, where both are .Vector3dVector
                        colorData
                    ]

                    where positionData is a list of vectors
                    [
                        [x1,y1,z1],
                        ...
                        n times for n agents
                    ]
                    likewise for the colors
                                      [
                        [R1,G1,B1],
                        ...
                        n times for n agents
                    ]

        :return:  NONE
        '''

        self.pcd.points = open3d.utility.Vector3dVector(frameData[0]) if not converted else frameData[0]
        self.pcd.colors = open3d.utility.Vector3dVector(frameData[1]) if not converted else frameData[1]
        self.vis.update_geometry()

        if self.firstFrame:
            self.vis.reset_view_point(True)
        self.vis.poll_events()
        self.vis.update_renderer()
        time.sleep(self.frameDelay)
        self.firstFrame = False
    def destroyWindow(self):
        '''
        :Description: Destroys the running window
        :return:
        '''
        self.vis.destroy_window()
    def runFrames(self,framesData):
        '''
        :description: Visualize all frames
        :param frameDatas: (a list of frameData ... see .tick method)
                [
                    [
                        positionData, where both are .Vector3dVector
                        colorData
                    ]
                ]

                    where positionData is a list of vectors
                    [
                        [x1,y1,z1],
                        ...
                        n times for n agents
                    ]
                    likewise for the colors
                                      [
                        [R1,G1,B1],
                        ...
                        n times for n agents
                    ]
        :return:  NONE
        '''
        try:
        # Pre-compute the frame conversion
            for indx in range(len(framesData)):
                pos = open3d.utility.Vector3dVector(list(framesData[indx][0]))
                colors = open3d.utility.Vector3dVector(list(framesData[indx][1]))
                framesData[indx] = [pos, colors]
        except:
            print(framesData[indx][1])

        # Display the simulation
        for i in range(0,len(framesData)):
            self.tick(framesData[i],converted=True)
        self.destroyWindow()




