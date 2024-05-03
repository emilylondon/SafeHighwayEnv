import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go

from odp.Grid import Grid
from odp.Plots.plotting_utilities import pre_plot, downsample

def safety_wrapper_plot_BRT(BRT):

    V_ori = BRT.result 
    grid = BRT.g

    dims_plot = BRT.po.dims_plot

    grid, my_V = pre_plot(BRT.po, grid, V_ori)

    print(f"len(my_V.shape) = {len(my_V.shape)}")

    if len(dims_plot) == 2 and len(my_V.shape) == 2:
        # Plot 2D isosurface for only one time step
        dim1, dim2 = dims_plot[0], dims_plot[1]
        complex_x = complex(0, grid.pts_each_dim[dim1])
        complex_y = complex(0, grid.pts_each_dim[dim2])
        mg_X, mg_Y = np.mgrid[grid.min[dim1]:grid.max[dim1]: complex_x, grid.min[dim2]:grid.max[dim2]: complex_y]


        if (my_V > 0.0).all():
            print("Implicit surface will not be shown since all values are positive ")
        if (my_V < 0.0).all():
            print("Implicit surface will not be shown since all values are negative ")

        print("Plotting beautiful 2D plots. Please wait\n")
        fig = go.Figure(data=go.Contour(
            x=mg_X.flatten(),
            y=mg_Y.flatten(),
            z=my_V.flatten(),
            zmin=0.0,
            ncontours=1,
            contours_coloring='none',  # former: lines
            name="Reachable Set",  # zero level
            line_width=1.5,
            line_color='magenta',
            zmax=0.0,
        ), layout=go.Layout(plot_bgcolor='rgba(0,0,0,0)'))  # ,paper_bgcolor='rgba(0,0,0,0)'

    if BRT.po.do_plot:
        fig.show()
        print("Please check the plot on your browser.")

    # Local figure save
    if BRT.po.save_fig:
        if BRT.po.interactive_html:
            fig.write_html(BRT.po.filename + ".html")
        else:
            fig.write_image(BRT.po.filename)
""" 

    if len(dims_plot) != 3 and len(dims_plot) != 2 and len(dims_plot) != 1:
        raise Exception('dims_plot length should be equal to 3, 2 or 1\n')

    print(f"len of dims_plot == {len(dims_plot)}, len of my_V == {len(my_V.shape)} and shape = {my_V.shape}")

    brt_perimeter = []
    for i in range(my_V.shape[0]):
        for j in range(my_V.shape[1]):
            value = my_V[i][j]
            print(f"({i}, {j}): {value}")
            if value <= 0:
                brt_perimeter.append((i, j))
    # close the shape 
    brt_perimeter.append((brt_perimeter[0]))
    brt_perimeter = np.array(brt_perimeter)

    print(f"brt_perimeter to visualize = {brt_perimeter}")
    # Plot the shape
    plt.plot(brt_perimeter[:,0], brt_perimeter[:,1], 'b-')  # Plot the perimeter
    plt.fill(brt_perimeter[:,0], brt_perimeter[:,1], 'lightblue')  # Fill the shape with color (optional)

    # Set plot labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Shape')

    # Set aspect ratio to equal
    #plt.gca().set_aspect('equal', adjustable='box')

    # Show plot
    plt.grid(True)
    plt.show()
    assert(False) """


""" 

def pre_plot(plot_option, grid, V_ori):
    #Pre-processing steps for plotting


    # Slicing process
    dims_plot = plot_option.dims_plot
    idx = [slice(None)] * grid.dims
    slice_idx = 0

    # Build new grid
    grid_min = grid.min
    grid_max = grid.max
    dims = grid.dims
    N = grid.pts_each_dim

    delete_idx = []
    dims_list = list(range(grid.dims))
    for i in dims_list:
        if i not in dims_plot:
            idx[i] = plot_option.slices[slice_idx]
            slice_idx += 1
            dims = dims -1
            delete_idx.append(i)
    N = np.delete(N, delete_idx)
    grid_min = np.delete(grid_min, delete_idx)
    grid_max = np.delete(grid_max, delete_idx)

    V = V_ori[tuple(idx)]
    grid = Grid(grid_min, grid_max, dims, N)

    # Downsamping process
    if plot_option.scale is not None:
        scale = plot_option.scale
    else:
        scale = [1] * grid.dims
        for i in range(grid.dims):
            if grid.pts_each_dim[i] > 30:
                scale[i] = np.floor(grid.pts_each_dim[i]/30).astype(int)
    grid, V = downsample(grid, V, scale)

    return grid, V

def downsample(g, data, scale):
    #Dowsampling for large 3D grid size, e.g. 100x100x100 for efficient plotting

    if len(scale) != g.dims:
        raise Exception('scale length should be equal to grid dimension\n')

    odd_ind =[False] * g.dims
    for i in range(g.dims):
        if g.pts_each_dim[i] % scale[i] != 0:
            odd_ind[i] = True
    
    # Generate new data
    idx = [slice(0,None,scale[0])] * g.dims
    for i in range(g.dims):
        if odd_ind[i]:
                idx[i] = slice(0,-(g.pts_each_dim[i]%scale[i]),scale[i])
    data_out = data[tuple(idx)]
    # Generate new grid
    grid_min = g.min
    grid_max = g.max
    dims = g.dims
    N = g.pts_each_dim
    for i in range(g.dims):
        if odd_ind[i]:
            grid_max[i] = g.max[i]-(g.pts_each_dim[i]%scale[i])*(g.max[i]-g.min[i])/g.pts_each_dim[i]
            N[i] = ((g.pts_each_dim[i]-g.pts_each_dim[i]%scale[i])/scale[i]).astype(np.int64)
        else:
            N[i] = (g.pts_each_dim[i]/scale[i]).astype(np.int64)
    g_out = Grid(grid_min, grid_max, dims, N)

    return g_out, data_out 
    # Close the perimeter to form a loop
    x_coords.append(x_coords[0])
    y_coords.append(y_coords[0])

    # Plot the shape
    plt.plot(x_coords, y_coords, 'b-')  # Plot the perimeter
    plt.fill(x_coords, y_coords, 'lightblue')  # Fill the shape with color (optional)

    # Set plot labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Shape')

    # Set aspect ratio to equal
    plt.gca().set_aspect('equal', adjustable='box')

    # Show plot
    plt.grid(True)
    plt.show()

# Example perimeter of a polygon specified by each point
polygon_x = [0, 1, 1.5, 1, 0]
polygon_y = [0, 0.5, 1, 1.5, 0]

# Plot the polygon
plot_shape(polygon_x, polygon_y)
 """