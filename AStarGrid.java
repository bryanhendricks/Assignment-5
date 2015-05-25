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
        		this.cells[row][col] = new AStar(0, 0, null);
        		col++;
        	}
        	row++;
        }
    }
    public AStar get_cell(Point point) {
        return this.cells[point.x][point.y];
    }
    public void set_cell(int row, int col, AStar a){
    	this.cells[row][col] = a;
    }
}