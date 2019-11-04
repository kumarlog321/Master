package com.codewithkumar;
import java.util.Scanner;

public class Main {

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        String ccnum = scanner.next();
        ccnum = maskCCNumber(ccnum);
    }

    private static String maskCCNumber(String ccnum) {
        int total = ccnum.length();
        int startlen=1,endlen = 4;
        String masked="";
        char  mask;
        StringBuffer maskedbuf = new StringBuffer(ccnum.substring(0,startlen));
        if(ccnum.length()>6 &&
                ((ccnum.chars().anyMatch(Character::isDigit)))) {
            int masklen = total-(startlen + endlen) ;
            for (int i = 1; i <= masklen; i++) {
                if ((Character.isDigit(ccnum.charAt(i))))
                    mask = '#';
                else
                    mask=ccnum.charAt(i);
                maskedbuf.append(mask);
            }
            maskedbuf.append(ccnum.substring(startlen + masklen, total));
             masked = maskedbuf.toString();
            System.out.println( masked );
            masked= masked;
        }
        return masked=ccnum;
    }
}
