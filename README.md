# Robot_Guidance_with_Head_Movements

Bu çalışmada omurilik zedelenmesi geçiren, doğuştan veya bir hastalıktan kaynaklanan sebeplerle tekerlekli sandalyeye bağımlı olan bireylerin hayatını kolaylaştırmak için 
minimum kafa hareketi veya arayüz üzerinden tekerlekli sandalyelerini kontrol edebilmeleri amacıyla tasarlanmıştır Tekerlekli sandalyeyi daha rahat kontrol edebilmeleri 
için bir kamera yardımıyla kullanıcıların kafa hareketlerini tespit etmek için öncelikle kullanıcının yüz bölgesinin tespit edilmesi gerekmektedir Dlib kütüphanesi kullanılarak 
algılanan yüz üzerinde önemli 68 nokta belirlenir ve bu belirlenen noktalardan birkaçı kullanılarak baş hareketinin algılanması sağlanır.Algılanan baş hareketi daha önceden 
belirlenen hareketlerden birine uyuyorsa tekerlekli sandalyenin gideceği hareket yönüne karar verilmiştir Tekerlekli sandalye karar verilen yön doğrultusunda uygun hareketi 
gerçekleştirecektir.

![ScreenShot](https://github.com/eelfgnc/Robot_Guidance_with_Head_Movements/arayuz.png?raw=true)

Kullanıcı tekerlekli sandalyeyi arayüz üzerinden 2 farklı şekilde kontrol edebilir. Arayüz üzerinde gösterilen haritada 3 yeşil button vardır ve bu butonlar ile gösterilen 
koordinatlara otonom bit şekilde hareket eder. Kullanıcı otonom hareket etmek istemez ise yön buttonları sayesinde tekerlekli sandalyeyi kontrol edebilir. 
