package com.codewithkumar;
import java.util.Stack;

public class Main {

    public static void main(String[] args) {
        String abc=" ";
        String[] tokens = abc.split(" ");
        double a = evalRPN(tokens);
        System.out.println(a);
    }

    public static double evalRPN(String[] tokens) {
               if(tokens==null||tokens.length<=0)
                        return 0;
                double ans = 0;
                Stack<Double> res = new Stack<Double>();
                for(int i = 0; i<tokens.length;i++){
                    ans = 0;
                        if(tokens[i].equals("/")||tokens[i].equals("*")||tokens[i].equals("+")||tokens[i].equals("-")){
                                double b = res.pop();
                                 double a = res.pop();
                                if(tokens[i].equals("/"))
                                    ans += a/b;
                                else if(tokens[i].equals("+"))
                                         ans += a+b;
                                else if(tokens[i].equals("-"))
                                         ans += a-b;
                                else if(tokens[i].equals("*"))
                                        ans += a*b;
                                 res.push(ans);
                            }else{
                            res.push(Double.parseDouble(tokens[i]));
                            }
                   }
               return res.pop();
            }


}
