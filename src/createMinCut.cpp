
#include "createMinCut.h"
#include <queue>
#include "extra.h"
#define _USE_MATH_DEFINES
#include <cmath>
using namespace cv;
using namespace std;

#define MAX1 200000
vector<vector<double> > energy;


#include<time.h>

double calculateMean(Area ar, vector<vector<double> > enr)
{
    //We are calculating mean of assigned Colour in config file
    double sum = 0.0;
    set<int> all_pixels = ar.getPixels();
    set<int>::iterator it= all_pixels.begin();
    while (it!= all_pixels.end())
    {
        int x = *it/ar.getImageColumns();
        int y = *it%ar.getImageColumns();
        sum = sum + enr[x][y];
        it++;
    }
    return sum/ ar.getPixelsCount();

}
double calculateVariance(Area ar, vector<vector<double> > energy, double mean)
{
    double sum = 0.0;
    set<int> all_pixels = ar.getPixels();
    long pixelsCount = ar.getPixelsCount();
    set<int>::iterator it = all_pixels.begin();
    while(it!= all_pixels.end())
    {
        int x = *it/ar.getImageColumns();
        int y = *it%ar.getImageColumns();
        sum = sum + pow((energy[x][y]-mean),2);
        it++; 
    }
    return sum/(pixelsCount - 1);
}
double calculateIndivualEnergy(Mat &in_image, int i, int j) {
    Vec3b pixel = in_image.at<Vec3b>(i, j);
    return (pixel[0] + pixel[1] + pixel[2]) / 3;
}
double gaussianProb(double en, double mean, double var)
{
    double res = 0.0;
    static const double value = 0.39904344223;
    if(var ==0)
    { 
        var = mean;
    }
    double a = pow((en - mean),2);
    return (value / sqrt(var)) * std::exp(-0.5d * (a/var));

}
void contactGraphWithMainSinkSource(Graph &graph, int rows, int cols, double fg_mean, double bg_mean, double fg_var, double bg_var, vector<vector<double> > energy,Area foreground ,Area background)
{
    int fg_source = rows*cols;
    int bg_source = (rows*cols) + 1;
    int ctr = 0;
    int ctr2 = 0;
    int fgPixel = rows * cols;
    int bgPixel = (rows * cols) + 1;
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            double fgProb = gaussianProb(energy[i][j],fg_mean,fg_var);
            double bgProb = gaussianProb(energy[i][j],bg_mean,bg_var);

            double fgW = fgProb/(fgProb+bgProb);
            double bgW = bgProb/(fgProb+bgProb);

            int pixel = i*cols + j;
            if (foreground.getPixels().count(pixel) > 0) {
                graph.addEdge(fgPixel,pixel,2,i,j);
                graph.addEdge(pixel,bgPixel,1,i,j);
                ctr++;

            } 
            else if (background.getPixels().count(pixel) > 0) {
                graph.addEdge(pixel,bgPixel,2,i,j);
                graph.addEdge(fgPixel,pixel,1,i,j);
                ctr2++;

            } 
            else if (fgW > bgW) {
                graph.addEdge(fgPixel,pixel,2,i,j);
                graph.addEdge(pixel,bgPixel,1,i,j);
                ctr++;

            } else {
                graph.addEdge(pixel,bgPixel,2,i,j);
                graph.addEdge(fgPixel,pixel,1,i,j);
                ctr2++;
            }
        }
    }
     
}
double getInterPixelDistance(Mat in_image, int x1, int y1, int x2, int y2) {
    Vec3b pixel1 = in_image.at<Vec3b>(x1, y1);
    Vec3b pixel2 = in_image.at<Vec3b>(x2, y2);

    return sqrt((pow(pixel1[0] - pixel2[0], 2) + pow(pixel1[1] - pixel2[1], 2) + pow(pixel1[2] - pixel2[2], 2)));
}
double calculateDistance(Mat input, int pixel1, int pixel2) {
    int x1 = pixel1 / input.cols;
    int y1 = pixel1 % input.cols;
    int x2 = pixel2 / input.cols;
    int y2 = pixel2 % input.cols;
    double pixelDistance = getInterPixelDistance(input, x1, y1, x2, y2);
    return pixelDistance;
}
void connectInterNode(Graph &graph, Mat &input, int rows, int cols)
{
    int count = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {

            int pixel = i * cols + j;

            if (i != rows - 1) {
                
                int dPixel = (i + 1) * cols + j;
                if (calculateDistance(input, pixel, dPixel) <= 0) {
                    count++;
                    graph.addEdge(pixel, dPixel, 2, i, j);
                } else {
                    count++;
                    graph.addEdge(pixel, dPixel,1, i, j);

                }
            }

            if (j != cols - 1) {
                
                int rPixel = pixel + 1;
                if (calculateDistance(input, pixel, rPixel) <= 0) {
                    count++;
                    graph.addEdge(pixel, rPixel, 2, i, j);
                } else {
                    count++;
                    graph.addEdge(pixel, rPixel, 1, i, j);

                }
            }
        }
    }
}
void intializePixelAllocation(int cols, int rows, Mat &input, Area foreground, Area background, Graph &graph)
{
    vector<vector<double> > temp_energy(rows);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            temp_energy[i].push_back(calculateIndivualEnergy(input, i, j));
        }
    }
    energy = temp_energy;

    double fg_mean = calculateMean(foreground, energy);
    double bg_mean = calculateMean(background, energy);
    double fg_var = calculateVariance(foreground, energy, fg_mean);
    double bg_var = calculateVariance(background, energy, bg_mean);
    contactGraphWithMainSinkSource(graph,rows,cols,fg_mean,bg_mean,fg_var,bg_var,energy,foreground,background);
    connectInterNode(graph, input, rows, cols);

}
void FindPath(Graph &res_gr, int src, int dest, bool *visited, vector<int> &path, int *check)
{
        path.push_back(src);
	if(*check == 1)
	{
		return;
	}
	visited[src] = true;
	
	if(src == dest)
	{
		*check = 1;	
		return;
	}	
	else
	{
        vector<Edge> &adjPixels = res_gr.getNearbyNodes(src);
        vector<Edge>::iterator it = adjPixels.begin();
        while (it != adjPixels.end()) {            
            Edge &e = *it;
            if(e.weight>0 && visited[e.vertices]==false)
            {
                FindPath(res_gr,e.vertices,dest,visited,path, check);
            }
            it++;
        }
	}
	visited[src] = false;
}
bool checkPath(Graph &res_gr, int src, int sink, vector<int> &path, int x)
{
    path.clear();
    int flag = 1;
	int check = 1;
    int V = res_gr.getVertices();
    bool visited[V];
	for(int i=0;i<V;i++)
	{
		visited[i] = false;
	}
    path.clear();
    path.push_back(src);
    FindPath(res_gr,x,sink,visited,path,&check);
    path.push_back(sink);
	if(flag == 1){
		return true;
	}
	return false;
	
}
Mat bfsTraversal(Graph &res_graph, Mat out_image, int source/*, bool visited[]*/) {

    Mat new_image = out_image.clone();
    int count=0;
    for(int i=0;i<res_graph.getNearbyNodes(source).size();i++)
    {
        if(res_graph.getNearbyNodes(source)[i].weight ==0)
        {
            int x= res_graph.getNearbyNodes(source)[i].pixelX;
            int y= res_graph.getNearbyNodes(source)[i].pixelY;
            Vec3b pixel;
            pixel[0] = 0;
            pixel[1] = 0;
            pixel[2] = 0;
            new_image.at<Vec3b>(x, y) = pixel;
            count++;
        }
	else
	{
            int x= res_graph.getNearbyNodes(source)[i].pixelX;
            int y= res_graph.getNearbyNodes(source)[i].pixelY;
            Vec3b pixel;
            pixel[0] = 255;
            pixel[1] = 255;
            pixel[2] = 255;
            new_image.at<Vec3b>(x, y) = pixel;
            count++;
	}
        
        
    }
    return new_image;
}
Mat maxflow(Graph &graph, Mat &output, int src, int sink, int rows, int cols)
{
    int V = graph.getVertices();
    vector<int> path;
    Graph res_graph = Graph((rows*cols)+2);
    for(int i=0;i<V;i++)
    {
        res_graph.enterNeigbouringNodes(graph.getNearbyNodes(i),i);
    }
    int final_flow = 0;
    int k = 0;
    clock_t begin = clock();
    vector<Edge> v = res_graph.getNearbyNodes(src);
    int min = 1;
    int count = 0;


    
    cout<<"START"<<endl;
    while(checkPath(res_graph, src, sink, path,k))
    {
        
        if(k == rows*cols+2)
        {
            break;
        }
        int min = 1;
        int ch;
        v[k].weight -= min;
        vector<Edge> m = res_graph.getNearbyNodes(k);
        for(int i=0;i<res_graph.getNearbyNodes(k).size();i++)
            {
                if(res_graph.getNearbyNodes(k)[i].weight!=0 && res_graph.getNearbyNodes(k)[i].vertices == sink)
                {
                    ch = i;
                    break;
                }
            }
        m[ch].weight -=min;
        
        final_flow = final_flow + min;
        k++;
    }
    res_graph.enterNeigbouringNodes(v,src);
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    cout << time_spent <<endl;
    cout<<final_flow<<endl;
    return bfsTraversal(res_graph,output,src);
    
}
Mat createGraph(Mat &output, Mat &input, Area foreground, Area background)
{
    
    int rows = input.rows;
    int cols = input.cols;
    Graph graph = Graph((rows*cols)+2);
    intializePixelAllocation(cols,rows,input,foreground,background,graph);
    int fgPixel = rows*cols;
    int bgPixel = rows*cols + 1;
    return maxflow(graph,output,fgPixel,bgPixel,rows,cols);

    
    

}
