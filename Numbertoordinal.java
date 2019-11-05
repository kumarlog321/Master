class Challenge {
  public static String numberToOrdinal( Integer number ) {
    while(number!=0){
    int mod100=number%100;
    int mod10=number%10;
    if(mod10==1 && mod100!=11){
      return number+"st";
    }
    else if(mod10==2 && mod100!=12){
      return number+"nd";
      
    }
    else if(mod10==3 && mod100!=13){
      return number+"rd";
      
    }
    else {
      return number+"th";
    }
  }
    return number+"";
  }
}
