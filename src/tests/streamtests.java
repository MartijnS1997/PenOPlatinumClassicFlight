package tests;

import internal.Helper.Vector;
import internal.Physics.PhysXEngine;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 19/03/2018.
 */
public class streamtests {
    @Test
    public void vectorAddLambda(){
        Vector baseVector = new Vector(5,5,5);
        List<Vector> vectors = new ArrayList<>();
        vectors.add(new Vector(0,0,0));
        vectors.add(new Vector(0,1,2));
        vectors.add(new Vector(3,2,1));
        vectors.add(new Vector(25,85,96));

        List<Vector> result =vectors.stream().map(v-> v.vectorSum(baseVector)).collect(Collectors.toList());
        System.out.println(result);
    }

    @Test
    public void vectorDistanceLambda(){
        List<Vector> pathList = new ArrayList<>();
        Vector startPos = new Vector();
        pathList.add(new Vector(5,0,0));
        pathList.add(new Vector(0,2,0));
        pathList.add(new Vector(0,0,3));
        Optional<Vector> closestPos = pathList.stream().reduce((closest, next)-> closest.distanceBetween(startPos) < next.distanceBetween(startPos) ? closest : next);
        System.out.println(closestPos.get());
    }

    @Test
    public void inFrontOfTest(){
        Vector cubePosition = new Vector(-1,25,-20);
        Vector dronePosition = new Vector(0,0,0);
        Vector droneOrientation = new Vector((float) (-20*PI/180), (float) (20*PI/180),0);
        Vector diffVectorWorld = cubePosition.vectorDifference(dronePosition);
        //transform it onto the drone axis system
        Vector diffVectorDrone = PhysXEngine.worldOnDrone(diffVectorWorld, droneOrientation);
        //then project the difference vector on the heading vector
        Vector headingVector = new Vector(0,0,-1);
        Vector projectedDiff = headingVector.projectOnVector(diffVectorDrone);
        System.out.println(projectedDiff);
        //check the scalar product of the projection, if positive we're behind, if not we're ahead
        System.out.println(projectedDiff.scalarProduct(headingVector) >= 0);
    }
}
