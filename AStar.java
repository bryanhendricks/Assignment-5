public class AStar{
	private int g;
	private int h;
	private Point came_from;
	public AStar(int g, int h, Point came_from){
		this.g = g;
		this.h = h;
		this.came_from = came_from;
	}
	public int get_g(){
		return this.g;
	}
	public int get_h(){
		return this.h;
	}
	public int get_f(){
		return this.g + this.h;
	}
	public Point came_from(){
		return this.came_from;
	}
}