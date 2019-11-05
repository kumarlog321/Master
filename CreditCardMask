package com.codewithkumar;
import java.util.Scanner;

public class Main {

    public static void main(String[] args) {
        //Scanner scanner = new Scanner(System.in);
       // String ccnum = scanner.next();
        String ccnum="1234567890";
        ccnum = maskCCNumber(ccnum);
        System.out.println(ccnum);
    }

    private static String maskCCNumber(String creditCardNumber) {
String masked="";
		int total = creditCardNumber.length();
		 if(creditCardNumber.length()!=0) {
			int startlen=1,endlen = 4;			
			char  mask;
			StringBuffer maskedbuf = new StringBuffer(creditCardNumber.substring(0,startlen));
			if(creditCardNumber.length()>=6 &&
					((creditCardNumber.chars().anyMatch(Character::isDigit)))) {
				int masklen = total-(startlen + endlen) ;
				for (int i = 1; i <= masklen; i++) {
					if ((Character.isDigit(creditCardNumber.charAt(i))))
						mask = '#';
					else
						mask=creditCardNumber.charAt(i);
					maskedbuf.append(mask);
				}
				maskedbuf.append(creditCardNumber.substring(startlen + masklen, total));
				 masked = maskedbuf.toString();
				//System.out.println( masked );
				masked= masked;
			}
			else
			 masked=creditCardNumber;
		 }
		else{
			masked=creditCardNumber;
		}
		return masked;
    }
}
