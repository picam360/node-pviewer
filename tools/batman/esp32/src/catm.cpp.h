
String getResponce(int wait_ms)
{
    String responce;
    unsigned long st = millis();
    while (millis() - st < wait_ms)
    {
        while (SerialAT.available())
        {
            char c = (char)SerialAT.read();
            if (c == '\n')
            {
                responce += "\\n";
            }
            else if (c == '\r')
            {
                responce += "\\r";
            }
            else
            {
                responce += c;
            }
        }
    }
    return responce;
}
void connectCATM()
{
    unsigned long start = millis();

    int step = 0;
    while (millis() - start < 3600000)
    {
        M5.Display.fillScreen(BLACK); // 画面を黒でクリア
        M5.Display.setTextSize(1);    // 文字サイズ設定
        M5.Display.setCursor(0, 0);   // 左上にカーソルセット
        M5.Display.println("Connect CAT-M...");
        M5.Display.print("STEP: ");
        M5.Display.println(step);

        if (step == 0)
        {
            if (!modem.init())
            {
                step++;
                continue;
            }
            if (!modem.waitForNetwork())
            {
                step++;
                continue;
            }
            if (!modem.isNetworkConnected())
            {
                step++;
                continue;
            }
            if (!modem.gprsConnect(config.CATM_APN.c_str(), config.CATM_USR.c_str(), config.CATM_PWD.c_str()))
            {
                step++;
                continue;
            }

            M5.Display.println("APN connected");
            delay(1000);
            return;
        }
        if (step == 1)
        {
            M5.Display.println("Restart Modem");
            modem.restart();
            step++;
            delay(1000);
        }
        else if (step == 2)
        {
            String mi = modem.getModemInfo();
            M5.Display.print("Modem: ");
            if (mi.isEmpty())
            {
                M5.Display.println("Wait Info");
                delay(1000);
                continue;
            }
            M5.Display.println(mi);
            step++;
            delay(1000);
        }
        else if (step == 3)
        {
            if (modem.getSimStatus() != 1)
            {
                M5.Display.println("Sim: Not READY");
                delay(1000);
                continue;
            }

            M5.Display.println("Sim: READY");
            step++;
            delay(1000);
        }
        else if (step == 4)
        {

            M5.Display.print("COPS: ");
            SerialAT.println("AT+COPS?");
            String cops = getResponce(1000);
            M5.Display.println(cops);

            int copsValue = cops.substring(cops.indexOf(':') + 1).toInt();
            if (copsValue != 0)
            {
                M5.Display.print("COPS: from ");
                M5.Display.print(copsValue);
                M5.Display.println(" to 0");

                modem.sendAT("+COPS=0");
                modem.waitResponse();
                delay(3000);
            }

            M5.Display.print("CGDCONT: ");
            SerialAT.println("AT+CGDCONT?");
            String cgdcont = getResponce(1000);
            M5.Display.println(cgdcont);

            if (cgdcont.indexOf("\"" + config.CATM_APN + "\"") == -1)
            {
                M5.Display.println("APN: " + config.CATM_APN);
                modem.sendAT("+CGDCONT=1,\"IP\",\"" + config.CATM_APN + "\"");
                modem.waitResponse();
                delay(3000);
            }

            // M5.Display.print("COPS: ");
            // SerialAT.println("AT+COPS=?");
            // while(!SerialAT.available()){
            //     delay(1000);
            //     M5.Display.print(".");
            // }
            // String copsq = getResponce(1000);
            // M5.Display.println(copsq);
            // USBSerial.println(copsq);

            // M5.Display.print("SET COPS: ");

            // //SerialAT.println("AT+COPS=1,2,\"44020\",7");//softbank
            // SerialAT.println("AT+COPS=1,2,\"44020\"");//softbank
            // while(!SerialAT.available()){
            //     delay(1000);
            //     M5.Display.print(".");
            // }
            // String set_cops = getResponce(1000);
            // M5.Display.print(set_cops);
            // USBSerial.println(set_cops);

            step++;
            delay(1000);
        }
        else if (step == 5)
        {
            M5.Display.println("FIX: CAT-M, LTE");

            M5.Display.print("CMNB: ");
            SerialAT.println("AT+CMNB?");
            String cmnb = getResponce(1000);
            M5.Display.println(cmnb);

            M5.Display.print("CNMP: ");
            SerialAT.println("AT+CNMP?");
            String cnmp = getResponce(1000);
            M5.Display.println(cnmp);

            int cmnbValue = cmnb.substring(cmnb.indexOf(':') + 1).toInt();
            int cnmpValue = cnmp.substring(cnmp.indexOf(':') + 1).toInt();
            if (cmnbValue != 1 || cnmpValue != 38)
            {

                M5.Display.print("CMNB: from ");
                M5.Display.print(cmnbValue);
                M5.Display.println(" to 1");

                M5.Display.print("CNMP: from ");
                M5.Display.print(cnmpValue);
                M5.Display.println(" to 38");

                modem.sendAT("+CFUN=0");
                modem.waitResponse();
                delay(3000);

                modem.sendAT("+CMNB=1");
                modem.waitResponse();
                delay(3000);

                modem.sendAT("+CNMP=38");
                modem.waitResponse();
                delay(3000);

                modem.sendAT("+CFUN=1");
                modem.waitResponse();
                delay(3000);
            }

            step++;
            delay(1000);
        }
        else if (step == 6)
        {
            // ④ 信号確認
            int16_t sq = modem.getSignalQuality();
            M5.Display.print("Signal: ");
            if (sq == 99)
            {
                M5.Display.println("Not Ready");

                M5.Display.print("CSQ: ");
                SerialAT.println("AT+CSQ");
                String csq = getResponce(1000);
                M5.Display.println(csq);

                M5.Display.print("CPIN: ");
                SerialAT.println("AT+CPIN?");
                String cpin = getResponce(1000);
                M5.Display.println(cpin);

                delay(1000);
                continue;
            }
            M5.Display.println(sq);
            M5.Display.println("Modem: Stable!");

            step++;
            delay(1000);
        }
        else if (step == 7)
        {
            M5.Display.print("waitForNetwork: ");
            if (!modem.waitForNetwork(3000))
            {
                M5.Display.println("Failed");

                M5.Display.print("CSQ: ");
                SerialAT.println("AT+CSQ");
                String csq = getResponce(1000);
                M5.Display.println(csq);

                M5.Display.print("CREG: ");
                SerialAT.println("AT+CREG?");
                String creg = getResponce(1000);
                M5.Display.println(creg);

                M5.Display.print("CEREG: ");
                SerialAT.println("AT+CEREG?");
                String cereg = getResponce(1000);
                M5.Display.println(cereg);

                M5.Display.print("CPSI: ");
                SerialAT.println("AT+CPSI?");
                String cpsi = getResponce(1000);
                M5.Display.println(cpsi);

                M5.Display.print("CMNB: ");
                SerialAT.println("AT+CMNB?");
                String cmnb = getResponce(1000);
                M5.Display.println(cmnb);

                M5.Display.print("CNMP: ");
                SerialAT.println("AT+CNMP?");
                String cnmp = getResponce(1000);
                M5.Display.println(cnmp);

                M5.Display.print("COPS: ");
                SerialAT.println("AT+COPS?");
                String cops = getResponce(1000);
                M5.Display.println(cops);

                M5.Display.print("CEER: ");
                SerialAT.println("AT+CEER");
                String ceer = getResponce(1000);
                M5.Display.println(ceer);

                delay(1000);
                continue;
            }

            if (!modem.isNetworkConnected())
            {
                M5.Display.println("Network not connected");
                delay(1000);
                continue;
            }

            M5.Display.println("Network connected");

            step++;
            delay(1000);
        }
        else if (step == 8)
        {
            M5.Display.println("Connecting to APN...");

            if (!modem.gprsConnect(config.CATM_APN.c_str(), config.CATM_USR.c_str(), config.CATM_PWD.c_str()))
            {
                M5.Display.print("CSQ: ");
                SerialAT.println("AT+CSQ");
                String csq = getResponce(1000);
                M5.Display.println(csq);

                delay(1000);
                continue;
            }
            M5.Display.println("APN connected");

            step++;
            delay(1000);
        }
        if (step == 9)
        {
            delay(1000);
            M5.Display.fillScreen(BLACK);
            return;
        }
    }
}