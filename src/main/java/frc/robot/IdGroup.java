package frc.robot;

public class IdGroup
{
    private final int[] ids;
    public IdGroup(int x, 
                   int... y)
    {
        ids = new int[y.length + 1];
        ids[0] = x;
        for (int i = 0; i < y.length; i++) 
        {
            ids[i + 1] = y[i];
        }                          
    }

    public int length()
    {
        return ids.length;
    }

    public int get(int index)
    {
        return ids[index];
    }

};