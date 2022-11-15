import java.io.FileNotFoundException;
import java.nio.file.Path;
import java.util.ArrayList;

public class Main {

    public static void main(String[] args) throws FileNotFoundException {

        PathFinder pf = new PathFinder();
        pf.readInput("input.txt");

        System.out.println("Shortest path distance: " + pf.distToDest(0, 3, 2));

        System.out.println("M-Path Distance: " + pf.distToDest(0, 9, 2));
        System.out.println("Number of shortest paths: " + pf.noOfShortestPaths(0, 3, 2));
        ArrayList<Integer> path = pf.fromSrcToDest(0, 3, 2);
        if (path == null)
            System.out.println("No path to destination");
        else {
            for (int i = 0; i < path.size(); i++)
                System.out.printf("%5s", path.get(i));
            System.out.println();
        }
    }
}
