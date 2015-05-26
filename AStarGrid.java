public class AStarGrid {
    protected int width;
    protected int height;
    private AStar[][] cells;
    public AStarGrid(int width, int height) {
        this.width = width;
        this.height = height;
        this.cells = new AStar[height][width];
        int row = 0;
        int col = 0;
        while(row < this.height){
        	while(col < this.width){
        		this.cells[row][col] = null;
        		col++;
        	}
        	row++;
        }
    }
    public AStar get_cell(Point point) {
    	if (this.cells[point.y][point.x] == null){
    		System.out.print("ERROR: AStar grid at ");
    		System.out.print(point.x);
    		System.out.print(", ");
    		System.out.print(point.y);
    		System.out.println(" is empty.");
    	}
        return this.cells[point.y][point.x];
    }
    public void set_cell(int row, int col, AStar a){
    	this.cells[row][col] = a;
    }
}