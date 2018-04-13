package tests;

import java.util.ArrayList;

import internal.Autopilot.Path;
import internal.Helper.Vector;
import internal.Testbed.WorldGenerator;

public class pathTest {


    private static Path p = new Path(null, null, null);

    private static int nbOfBlocks = 3;
    private static WorldGenerator wg = new WorldGenerator(nbOfBlocks);

    public static void main(String[] args) {
        ArrayList<Vector> pos = wg.allPositionsGenerator();

        p.convertCubeLocsToPath(pos);

        for (int i = 0; i < pos.size(); i++)
            System.out.println(pos.get(i).getxValue());
        System.out.println(java.util.Arrays.toString(p.getX()));

        for (int i = 0; i < pos.size(); i++)
            System.out.println(pos.get(i).getyValue());
        System.out.println(java.util.Arrays.toString(p.getY()));

        for (int i = 0; i < pos.size(); i++)
            System.out.println(pos.get(i).getzValue());
        System.out.println(java.util.Arrays.toString(p.getZ()));

    }

}
