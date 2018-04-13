package internal.Autopilot;


import internal.Testbed.World;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
/**
 * 
 * A class of the A* pathfinding algorithm
 * 
 * @author r0630364
 * @version 2.0
 *
 */
public class Pathfinding {
	
	/**
	 * 
	 * @param world
	 * 		the world where the unit is living in
	 * @post new.x = world.getX();
	 * @post new.y = world.getY();
	 * @post new.z = world.getZ();
	 * @post new.world = world;
	 */
	public Pathfinding(World world){
		X = world.getXsize();
		Y = world.getYsize();
		Z = world.getZsize();
		this.world = world;
	}
	
	private World world;
	private int X, Y, Z;	
	/**
	 * 
	 * @param start
	 * 		the start position
	 * @param end
	 * 		the end position
	 * @return
	 * 		the path
	 * @throws IOException
	 * 			if the dimensions of the position are not equal to 3
	 */
	public List<int[]> searchPath(int[] start, int[] end) throws IOException{
		
		
		ArrayList<N> Checked = new ArrayList<N>();
		
		PriorityQueue<N> positionQueue = new PriorityQueue<N>(new Comparator<N>(){
			@Override
			public int compare(N o1, N o2) {
				int cX = o1.getPosition()[0], cY = o1.getPosition()[1], cZ = o1.getPosition()[2];
				int dX = o2.getPosition()[0], dY = o2.getPosition()[1], dZ = o2.getPosition()[2];
				
				double distance1 = Math.sqrt(Math.pow((cX-end[0]),2) + Math.pow((cY-end[1]),2) + Math.pow((cZ-end[2]),2));
				double distance2 = Math.sqrt(Math.pow((dX-end[0]),2) + Math.pow((dY-end[1]),2) + Math.pow((dZ-end[2]),2));
				
				return Double.compare(distance1, distance2);
				
			}			
		});
		
		
		N lastN = null;		
		positionQueue.add(new N(start, null));		
		while (!positionQueue.isEmpty()){			
			N p = positionQueue.remove();			
			int[] pos = p.getPosition();	
			if (Arrays.equals(pos, end)){
				lastN = p;
				break;
			}
			List<int[]> neighbours = getNeighbours(p.getPosition());			
			for (int[] i : neighbours){				
				N n = new N(i, p);				
				if (!Checked.contains(n) && !positionQueue.contains(n)){					
					positionQueue.add(n);
				}
			}			
			Checked.add(p);
		}
		
		if (lastN != null){			
			ArrayList<int[]> path = new ArrayList<int[]>();			
			N currentN = lastN;			
			while(currentN != null){				
				path.add(currentN.getPosition());				
				currentN = currentN.getPrevious();
			}			
			Collections.reverse(path);			
			return path;
		}
		else
			return null;
		
		
	}

	
	private List<int[]> getNeighbours(int[] currentPos){		
		List<int[]> neighbours = new ArrayList<int[]>();
		for(int i = -1; i <= 1; i++){
			for (int j = -1; j <= 1; j++){
				for (int k= -1; k <= 1; k++){
					int[] pos = new int[]{currentPos[0]+i,currentPos[1]+j,currentPos[2]+k};
					if (0 <= pos[0] && pos[0] < X && 0 <= pos[1] && pos[1] < Y && 0 <= pos[2] && pos[2] < Z && isNeighbour(pos, currentPos)/**&& (world.isPassableTerrain(pos)||pos[2]==0) && unit.isNeighbouringImPassableTerrain(pos)**/){
						neighbours.add(pos);
					}
				}
			}
		}
		return neighbours;
	}
	/**
	 * 
	 * check whether the position me is a neighbour of the other position.
	 * 
	 * @param	me
	 * 			The first position to check.
	 * @param	other
	 * 			The second position to check.
	 * @return whether the position are neighbours.
	 * 		|if(me.isneibourof(other))
	 * 		|	result == true
	 * 		|else
	 * 		| 	result == false
	 * 
	 */
	public boolean isNeighbour (int[] me, int[] other){
		return (((Math.abs(me[0]-other[0]) == 1)||(me[0]-other[0]) == 0)
				&&((Math.abs(me[1]-other[1]) == 1)||((me[1]-other[1]) == 0))
				&&(((me[0]-other[0]) != 0)||((me[1]-other[1]) != 0)||((me[2]-other[2]) != 0))
				&&((Math.abs(me[2]-other[2]) == 1)||(me[2]-other[2]) == 0));
	}

	/**
	 * A class of nodes
	 * 
	 * @author r0630364
	 * @version 1.0
	 *
	 */
	private class N{
		
		private final int[] position;
		private final N previous;
		
		/**
		 * 
		 * @param pos
		 * 			the position of the node
		 * @param previous
		 * 			the previous node
		 * @throws IOException
		 * 			if the dimensions of the position are not equal to 3
		 */
		public N(int[] pos, N previous) throws IOException{
			if (pos.length != 3) throw new IOException("Invalid position length given to N");
			position = pos;
			this.previous = previous;
		}

		/**
		 * 
		 * @return the previous node
		 */
		public N getPrevious() {
			return previous;
		}

		/**
		 * 
		 * @return the position of the node
		 */
		public int[] getPosition() {
			return position;
		}
		
		@Override
		public String toString(){
			return Arrays.toString(position);
		}
		
		@Override
		public boolean equals(Object obj){			
			if (! (obj instanceof N)) 
				return false;			
			N node = (N)obj;			
			return (position[0] == node.getPosition()[0] && position[1] == node.getPosition()[1] && position[2] == node.getPosition()[2]);
		}
		
	}
	
}
