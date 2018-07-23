function outer

    inner1;
    disp(a);
    inner2;
    
    function inner1
       a = 5; 
    end

    function inner2
        disp(a);
        a = 6;
    end


end