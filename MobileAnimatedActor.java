import processing.core.PApplet;
import processing.core.PImage;

import java.util.Collections;
import java.util.List;
import java.util.ArrayList;

import static java.lang.Math.abs;

import java.lang.Math;

public abstract class MobileAnimatedActor
   extends AnimatedActor
{
	private ArrayList<Point> aPath;
	private ArrayList<Point> aChecked;
   public MobileAnimatedActor(String name, Point position, int rate,
      int animation_rate, List<PImage> imgs)
   {
      super(name, position, rate, animation_rate, imgs);
      aPath = new ArrayList<Point>();
      aChecked = new ArrayList<Point>();
   }
   public ArrayList<Point> get_path(){
	   return aPath;
   }
   public ArrayList<Point> get_checked(){
	   return aChecked;
   }
   public void resetPaths(){
	   aPath = new ArrayList<Point>();
	   aChecked = new ArrayList<Point>();
   }
   
   
   public int get_Dist(Point start, Point end){
	   int x_val;
	   int y_val;
	   if(start.x > end.x){
		   x_val = start.x - end.x;
	   }
	   else{
		   x_val = end.x - start.x;
	   }
	   if(start.y > end.y){
		   y_val = start.y - end.y;
	   }
	   else{
		   y_val = end.y - start.y;
	   }
	   return x_val + y_val;
   }
   
   public int get_F(Point pt, AStarGrid grid){
	   return grid.get_cell(pt).get_f();
   }
   
   public ArrayList<Point> reconstruct_path(AStarGrid grid, Point start, Point goal){
	   ArrayList<Point> path = new ArrayList<Point>();
	   Point cur = goal;
	   path.add(goal);
	   while (cur.x != start.x || cur.y != start.y){
		   int x = grid.get_cell(cur).came_from().x;
		   int y = grid.get_cell(cur).came_from().y;
		   cur = new Point(x, y);
		   aPath.add(cur);
		   path.add(cur);
	   }
	   
	   return path;
   }
   
   public ArrayList<Point> neighbors(WorldModel world, Point p, Point goal){
	   Point left = new Point(p.x - 1, p.y);
	   Point right = new Point(p.x + 1, p.y);
	   Point up = new Point(p.x, p.y - 1);                    //Is - upwards?
	   Point down = new Point(p.x, p.y + 1);

	   ArrayList<Point> n = new ArrayList<Point>();
	   
	   if (canPassThrough(world, left) && left.x >= 0){
		   n.add(left);
	   }
	   if (canPassThrough(world, right) && right.x <= 39){
		   n.add(right);
	   }
	   if (canPassThrough(world, up) && up.y >= 0){
		   n.add(up);
	   }
	   if (canPassThrough(world, down) && down.y <= 29){
		   n.add(down);
	   }
	   
	   if (left.x == goal.x && left.y == goal.y){
		   n.add(left);
	   }
	   if (right.x == goal.x && right.y == goal.y){
		   n.add(right);
	   }
	   if (up.x == goal.x && up.y == goal.y){
		   n.add(up);
	   }
	   if (down.x == goal.x && down.y == goal.y){
		   n.add(down);
	   }
	   
	   return n;
   }
   
   
   protected ArrayList<Point> a_Star(Point start, Point goal, WorldModel world){
//2: Initialize the sets, came_from, grid, and variables
	   ArrayList<Point> closedset = new ArrayList<Point>();
	   ArrayList<Point> openset = new ArrayList<Point>();
	   openset.add(start);
	   Point came_from = null;
	   AStarGrid grid = new AStarGrid(world.getNumCols(), world.getNumRows());
	   grid.set_cell(start.y, start.x, new AStar(0, get_Dist(start, goal), start));
	   while (openset.size() != 0){
		   int lowestF = 0;
		   int i = 0;
		   
//3: Steps through openset, finding the one with the lowest F
		   for (Point p : openset){
			   if (get_F(p, grid) < get_F(openset.get(lowestF), grid)){
				   lowestF = i;
			   }
			   i++;
		   }

//4: Sets current to the node in openset with the lowest F
		   Point current = openset.get(lowestF);
		   
//5: Checks to see if current is at the goal
		   if (current.x == goal.x && current.y == goal.y){
			   
//Final step: reconstructs and returns the path based on the a_star grid's came_from value
			   /*
			   for(Point p : reconstruct_path(grid, start, goal)){
				   System.out.print("x = ");
				   System.out.print(p.x);
				   System.out.print(" y = ");
				   System.out.println(p.y);
				   
			   }
			   System.out.println("");
			   System.out.println("");
			   System.out.println("");
			   */
			   return reconstruct_path(grid, start, goal);
		   }

//6: Moves the current node from openset to closedset; marked
		   openset.remove(current);
		   closedset.add(current);
		   aChecked.add(current);
			   
			   
//7: Steps through each neighbor of current
		   for (Point n : neighbors(world, current, goal)){
			   boolean closedBool = false;
			   
//8: If any of the neighbors are in the closed set, skip them
			   for(Point c : closedset){
				   if (n.x == c.x && n.y == c.y){
					   closedBool = true;
				   }
			   }
			   if (closedBool == true){
				   continue;
			   }
			   
			   grid.set_cell(n.y, n.x, new AStar(grid.get_cell(current).get_g() + 1,
					   get_Dist(n, goal), null));
			   
//9: Calculates tentative g_score of the current node
			   int tentative_g = grid.get_cell(current).get_g() + 1;
			   
//10: If the tentative g_score is less than the g_score of the neighbor
			   if (closedBool == false || tentative_g < grid.get_cell(n).get_g()){
				   
//11: Set the cell in the a_star grid at the neighbor node, with a came_from value of current
				   grid.set_cell(n.y, n.x, new AStar(grid.get_cell(current).get_g() + 1,
						   get_Dist(n, goal), current));
				   
//12: If the neighbor isn't in the openset, add them to it
				   int neighborVal = 0;
				   for (Point o : openset){
					   if (n.x == o.x && n.y == o.y){
						   neighborVal = 1;
					   }
				   }
				   if (neighborVal == 0){
					   openset.add(n);
				   }
			   }
		   }
	   }
	   System.out.println("ERROR: Goal not found.");
	   return new ArrayList<Point>();
   }

   protected Point nextPosition(WorldModel world, Point dest_pt)
   {
//1: Run a_Star, giving it the entity's position, destination point, and worldmodel
	   ArrayList<Point> path = a_Star(getPosition(), dest_pt, world);
	   if (path.size() == 2){
		   return path.get(1);
	   }
	   if (path.size() == 1){
		   return dest_pt;
	   }
	   
	   int i = path.size() - 2;

	   return path.get(i);
   }

   protected static boolean adjacent(Point p1, Point p2)
   {
      return (p1.x == p2.x && abs(p1.y - p2.y) == 1) ||
         (p1.y == p2.y && abs(p1.x - p2.x) == 1);
   }

   protected abstract boolean canPassThrough(WorldModel world, Point new_pt);
}
