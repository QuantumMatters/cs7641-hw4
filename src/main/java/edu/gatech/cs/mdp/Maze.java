/*
Inspired by Santiago Valdarrama's work on this assignment:
https://github.com/svpino/cs7641-assignment4/blob/master/assignment4/src/main/java/edu/gatech/cs7641/assignment4/artifacts/Problem.java
*/

package edu.gatech.cs.mdp;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;

public class Maze {

	private int[][] map;
	private int[] start;
    private int[] goal;
    private List<int[]> hazards;
    private int width;
    private int height;

    public static List<List<String>> readMaze(String path) {
        List<List<String>> records = new ArrayList<List<String>>();

        try (CSVReader csvReader = new CSVReader(new FileReader(path));) {
            String[] values = null;
            while ((values = csvReader.readNext()) != null) {
                records.add(Arrays.asList(values));
            }
        } catch (CsvValidationException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return records;
    }

    public Maze(String path) {
        List<List<String>> maze = readMaze(path);
        this.height = maze.size();
        this.width = maze.get(0).size();
        this.map = new int[this.width][this.height];
        this.start = new int[2];
        this.goal = new int[2];
        this.hazards = new ArrayList<int[]>();
        
        for (int i = 0; i < this.height; i++) {
            for (int j = 0; j < this.width; j++) {
                int x = j;
				int y = this.height - 1 - i;
                String c = maze.get(i).get(j);
                this.map[x][y] = 0;  // notice we flip the order here since BURLAP's flipped
                switch (c) {
                    case "1":
                        this.map[x][y] = 1;
                        break;
                    case "s":
                        this.start = new int[] {x, y};
                        break;
                    case "g":
                        this.goal = new int[] {x, y};
                        break;
                    case "h":
                        this.hazards.add(new int[] {x, y});
                        break;
                }
            }
        }
    }

    public int[] getStart() {
        return this.start;
    }

    public int[] getGoal() {
        return this.goal;
    }

    public int[][] getMap() {
        return this.map;
    }

    public List<int[]> getHazards() {
        return this.hazards;
    }

    public int getWidth() {
        return this.width;
    }

    public int getHeight() {
        return this.height;
    }

}
